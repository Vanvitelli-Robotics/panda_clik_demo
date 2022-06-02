#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_srvs/SetBool.h"
#include "sensor_msgs/JointState.h"

#include <Eigen/Dense>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


/*
    Funzione che assicura la continuità del quaternione.
    Dato un quaternione Q, si ha che -Q rappresenta la stessa rotazione.
    Quando si trasforma una matrice di rotazione in un Quaternione potrei avere indistintamente +Q o -Q.
    Questo crea un problema nell'inversione cinemarica che si può ritrovare un errore discontinuo.
    Questa funzione prende il quaternione q (al passo corrente) e prende il quaternione oldQ (al passo precendete)
    e restituisce un quaternione "equivalente a q" (rappresenta la stessa rotazione) ma "continuo" rispetto ad oldQ

    Non ci interessano i dettagli implementativi di questa funzione, è stata data nel pdf.
*/
Eigen::Quaterniond quaternionContinuity(const Eigen::Quaterniond &q, const Eigen::Quaterniond &oldQ)
{
    auto tmp = q.vec().transpose() * oldQ.vec();
    if (tmp < -0.01)
    {
        Eigen::Quaterniond out(q);
        out.vec() = -out.vec();
        out.w() = -out.w();
        return out;
    }
    return q;
}

//#####################################
/*
    Qualche variabile globale utile
*/
bool b_clik_run = false;                                           // booleano che indica se il clik è attivo o meno (verrà settato da un servizio)
moveit::core::RobotStatePtr kinematic_state;                       // lo stato cinematico del robot, possiamo leggerlo come lo stato dell'algoritmo CLIK (le q - variabili di giunto)
Eigen::Quaterniond oldQuaternion = Eigen::Quaterniond::Identity(); // oldQ usato per la continuità del quaternione, Inizializzato all'identità.
//#####################################

//#####################################
/*
    Callbk per la lettura della posa desiderata
    Nota l'utilizzo delle variabili globali per "comunicare" con il ciclo pricipale la posa desiderata.
*/
Eigen::Vector3d position_des;
Eigen::Quaterniond quaternion_des;
void pose_des_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    position_des(0) = msg->pose.position.x;
    position_des(1) = msg->pose.position.y;
    position_des(2) = msg->pose.position.z;

    quaternion_des.x() = msg->pose.orientation.x;
    quaternion_des.y() = msg->pose.orientation.y;
    quaternion_des.z() = msg->pose.orientation.z;
    quaternion_des.w() = msg->pose.orientation.w;

    // assicuro che il quaternione che ricevo come desiderato è "continuo" con lo stato attuale del robot nel nodo clik
    quaternion_des = quaternionContinuity(quaternion_des, oldQuaternion);
}

/*
    Callbk per la lettura della velocità (Twist) desiderato
    Nota l'utilizzo delle variabili globali per "comunicare" con il ciclo pricipale la posa desiderata.
    In generatore di traiettoria potrebbe restituire anche la velocità, in questo caso il CLIK potrebbe beneficiare
    della conoscenza di x_dot (la velocità della traiettoria) per implementare l'azione in feedforward.
    In questo esempio il generatore non pubblica la velocità e b_vel_des=ZERO sempre.
    L'algoritmo CLIK funziona anche così, ma potrebbe essere interessante implementare nel generatore di traiettoria
    la pubblicazione anche del Twist.
*/
Eigen::Matrix<double, 6, 1> b_vel_des;
void twist_des_cb(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
    b_vel_des(0) = msg->twist.linear.x;
    b_vel_des(1) = msg->twist.linear.y;
    b_vel_des(2) = msg->twist.linear.z;
    b_vel_des(3) = msg->twist.angular.x;
    b_vel_des(4) = msg->twist.angular.y;
    b_vel_des(5) = msg->twist.angular.z;
}
//#####################################

//#####################################
/*
    Callbk del servizio di attivazione del CLIK
    Il nodo di default non fa nulla (b_clik_run=false)
    Questo servizio permette di attivare il nodo (setta b_clik_run=true) e contestualmente lo inizializza

    *LA FASE DI INIZIALIZZAZIONE E' IMPORTANTE*
    quando viene richiesto di settare b_clik_run=True bisogna:
    - Settare lo stato del clik (memorizzato in kinematic_state sopra) ai valori correnti del robot vero.
    - Settare la posa desiderata (position_des, quaternion_des) al valore corrente di cinematica diretta (in modo da avere un errore iniziale nullo)
    - Settare il valore di azione in avanti (b_vel_des) a zero
    Il motivo per cui inizializziamo anche gli "input" del CLIK (position_des, quaternion_des,b_vel_des) è perchè l'arrivo del messaggio di comando
    non è istantaneo, tra l'attivazione del CLIK e l'arrivo del primo comando il robot non deve muoversi (l'errore del clik è zero)
*/
bool set_run_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
    // Sfrutto il servizio standard SetBool per ricevere la richiesta si start (true) o stop (false)

    if (!req.data) // <- se ho rishiesto stop
    {
        // semplicemente setto b_clik_run a false e ritorno.
        b_clik_run = false;
        ROS_INFO_STREAM("CLIK STOP");
        res.message = "stop";
        res.success = true;
        return true;
    }

    // se sono qui è stato richiesto uno start.

    // Devo leggere i valori correnti delle variabili di giunto per inizializzare il clik
    // in questo esempio uso ros::topic::waitForMessage per leggere un singolo messaggio dal topic /joint_states
    // ricorda che 'auto' serve per dire al compilatore di capire automaticamente il tipo (dovrebbe essre sensor_msgs::JointState::ConstPtr)
    ROS_INFO_STREAM("CLIK waiting joint state...");
    auto joint_msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");

    // aggiorno la variabile globale kinematic_state con i valori letti delle variabili di giunto
    for (int i = 0; i < joint_msg->name.size(); ++i)
    {
        kinematic_state->setJointPositions(joint_msg->name[i], &joint_msg->position[i]);
    }

    //####################
    // Qusesto blocco esegue la cinematica diretta (kinematic_state->getGlobalLinkTransform) e inizializza i valori desiderati
    // in modo da avere errore iniziale nullo
    // Le prime due righe servono per ottenere il link rispetto al quale fare la cinematica diretta (rivedi il tutorial di MoveIt sul RobotState)
    const moveit::core::JointModelGroup *joint_model_group = kinematic_state->getRobotModel()->getJointModelGroup("panda_arm");
    const moveit::core::LinkModel *last_link = kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back());
    const Eigen::Isometry3d &b_T_e = kinematic_state->getGlobalLinkTransform(last_link);
    position_des = b_T_e.translation();
    quaternion_des = b_T_e.rotation();
    oldQuaternion = quaternion_des;
    b_vel_des.setZero();
    //####################

    //#####################
    // Piccolo check di debug. Stampo le variabili di giunto appena salvate in kinematic_state.
    Eigen::VectorXd q;
    kinematic_state->copyJointGroupPositions(joint_model_group, q);
    ROS_INFO_STREAM("CLIK start! - q:\n"
                    << q);
    ROS_INFO_STREAM("position_des:\n"
                    << position_des);
    ROS_INFO_STREAM("quaternion_des:\n"
                    << quaternion_des.w() << " - " << quaternion_des.vec());

    for (int i = 0; i < joint_model_group->getJointModelNames().size(); i++)
        ROS_INFO_STREAM("getJointModelNames:[" << i << "]:" << joint_model_group->getJointModelNames()[i]);
    //#####################

    // finalmente setto b_clik_run a true e ritorno il servizio
    b_clik_run = true;

    res.success = true;
    return true;
}

/*
    Il main
*/
int main(int argc, char *argv[])
{
    // init e node handle
    ros::init(argc, argv, "clik");
    ros::NodeHandle nh;

    // Parametri del CLIK.
    // Tempo di campionamento e guadagno.
    double Ts = 0.001;
    double clik_gain = 1.0 / Ts;

    //#######################################
    /*
        Alcune variabili che mi servono per usare la libreria moveit (RobotState)
        - costruisco il kinematic_model
        - costruisco la variabile globale kinematic_state
        - definisco il 'last_link', il link da usare come end effector. In questo esempio scelgo l'ultimo link del gruppo panda_arm
    */
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    const moveit::core::RobotModelPtr &kinematic_model = robot_model_loader.getModel();
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
    const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("panda_arm");
    kinematic_state = moveit::core::RobotStatePtr(new moveit::core::RobotState(kinematic_model));
    const moveit::core::LinkModel *last_link = kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back());
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0); // <-- mi servirà per calcolare lo Jacobiano
    //#######################################

    // Inizializzo i subscriber
    ros::Subscriber pose_sub = nh.subscribe("clik/desired_pose", 1, pose_des_cb);
    ros::Subscriber twist_sub = nh.subscribe("clik/desired_twist", 1, twist_des_cb); // ricorda che in questo esempio non riceviamo veramente il twist desiderato

    // Inizializzo il servizio di set_run del clik
    ros::ServiceServer srv_set_run = nh.advertiseService("clik/set_run", set_run_cb);

    // Il publisher per i comandi in giunto
    ros::Publisher cmd_pub = nh.advertise<sensor_msgs::JointState>("/cmd/joint_position", 100);

    // Il ciclo principale, ricorda che DEVE girare con periodo Ts!
    ros::Rate loop_rate(1.0 / Ts);
    while (ros::ok())
    {

        // sleep sul rate per garantire la frequenza giusta (1/Ts)
        loop_rate.sleep();
        // spinOnce per eseguire eventuali chiamate a servizio o callbk di messaggi in arrivo
        ros::spinOnce();

        // Se b_clik_run è falso non devo fare nulla, torno all'inizio del cilo while
        if (!b_clik_run)
        {
            continue; // <-- mi andare direttamente alla prissima iterazione del ciclo
        }

        //###########################################################
        // Calcolo l'errore cinematico

        // Cinematica diretta
        const Eigen::Isometry3d &b_T_e = kinematic_state->getGlobalLinkTransform(last_link);

        // Estraggo la posizione
        Eigen::Vector3d position = b_T_e.translation();
        // Estraggo il quaternione
        Eigen::Matrix3d rotation = b_T_e.rotation();
        Eigen::Quaterniond quaternion(rotation);
        // Assicuro la continuità del quaternione
        quaternion = quaternionContinuity(quaternion, oldQuaternion);
        oldQuaternion = quaternion; // <-- per il prissimo ciclo

        // Calcolo l'errore 6D
        // - userò il metodo block delle matrici Eigen, potrei accedere anche ai
        //   singoli elementi del vettore usando qualche riga di codice in più
        Eigen::Matrix<double, 6, 1> error;

        // errore in posizione (setto il blocco di dimensioni 3x1 che parte dalla posizione (0,0))
        error.block<3, 1>(0, 0) = position_des - position;
        // Errore in orientamento è la parte vettoriale di Qd*inv(Q)
        Eigen::Quaterniond deltaQ = quaternion_des * quaternion.inverse();
        // errore in orientamento (setto il blocco di dimensioni 3x1 che parte dalla posizione (3,0))
        error.block<3, 1>(3, 0) = deltaQ.vec();
        //###########################################################

        //###########################################################
        // Calcolo le velocità di giunto con la seguente formula
        // qdot = Jpinv * (v_des + gamma*error);
        //  - Jpinv è la pseudoinversa dello Jacobiano
        //  - v_des è la velocità della traiettoria (azione in avanti, è zero in questo esempio...)
        //  - error è l'errore calcolato al passo precendete e gamma è il guadagno

        // vel_e è il termine (v_des + gamma*error)
        Eigen::Matrix<double, 6, 1> vel_e =
            b_vel_des + clik_gain * error;

        // Calcolo lo jacobiano
        Eigen::MatrixXd jacobian;
        kinematic_state->getJacobian(joint_model_group,
                                     last_link,
                                     reference_point_position, jacobian);

        // Calcolo pinv(jacobian)*vel_e, ovvero q_dot
        Eigen::VectorXd q_dot = jacobian.completeOrthogonalDecomposition().solve(vel_e);
        //###########################################################

        //###########################################################
        // Devo passare da q_dot a q -----> devo integrare.
        // Utilizzo una integrazione alla Eulero: q(k+1) = q(k) + T*q_dot(k)

        // copio i valori attuali del kinematic state (q(k))
        Eigen::VectorXd q;
        kinematic_state->copyJointGroupPositions(joint_model_group, q);

        // aggiorno q con la formula di Eulero (ottengo q(k+1))
        q = q + q_dot * Ts;

        // copio i nuovi valori di q nell'oggetto kinematic_state
        kinematic_state->setJointGroupPositions(joint_model_group, q);
        //###########################################################

        //###########################################################
        // Finalmente pubblico il risultato

        // definisco i joint state di comando copiando le position dal kinematic state
        sensor_msgs::JointState out_msg;
        kinematic_state->copyJointGroupPositions(joint_model_group, out_msg.position);
        // infine copio i nomi dei giunti dal joint_model_group (PASSAGGIO FONDAMENTALE)
        out_msg.name = joint_model_group->getJointModelNames();

        // riempio lo stamp, utile in caso di plot
        out_msg.header.stamp = ros::Time::now();

        cmd_pub.publish(out_msg);
        //###########################################################
        
    }

    return 0;
}
