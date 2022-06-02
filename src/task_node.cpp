#include "ros/ros.h"

#include "actionlib/client/simple_action_client.h"
#include "panda_clik_demo/JointTrajAction.h"
#include "panda_clik_demo/CartesianTrajAction.h"
#include "std_srvs/SetBool.h"

#include <tf2_ros/transform_listener.h>

/*
    Il nodo 'task' che coordina tutta la rete ROS.
    Può essere visto come uno script che fa vari passi

    1) Porta il robot nello spazio giunti in una configurazione di partenza chiamando l'azione di traiettoria nello spazio giunti
    2) Attiva il nodo CLIK usando il servizio di attivazione (il robot sarà fermo perchè l'errore iniziale del CLIK è zero)
    3) Legge la posa iniziale del l'link di end effector usando TF
    4) Chiama l'azione in spazio cartesiano passando la posa iniziale e finale
    5) Stop del CLIK
*/
int main(int argc, char *argv[])
{
    // ROS INIT e NodeHandle
    ros::init(argc, argv, "task_node");
    ros::NodeHandle nh;

    // Inizializzo gli action client di traiettoria cartesiana e in giunti
    actionlib::SimpleActionClient<panda_clik_demo::JointTrajAction> joint_client(nh, "/joint_traj_action", true);
    actionlib::SimpleActionClient<panda_clik_demo::CartesianTrajAction> cartesian_client(nh, "/cartesian_traj_action", true);

    // service client per attivare il clik
    ros::ServiceClient set_run_clik_cli = nh.serviceClient<std_srvs::SetBool>("clik/set_run");

    // Aspetta che i Server siano UP
    joint_client.waitForServer();
    cartesian_client.waitForServer();
    set_run_clik_cli.waitForExistence();

    // piccola variabile di supporto per fermare la demo nei vari passi
    char ans;

    // Traiettoria spazio giunti
    {
        // Attesa dell'utente
        ROS_INFO_STREAM("Traiettoria spazio giunti... - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;

        // Definizione della q_finale, il valore è scelto per portare il robot in una configurazione "destra" non singolare
        panda_clik_demo::JointTrajGoal joint_goal;
        joint_goal.duration = 3.0;
        joint_goal.qf = {0.0, 0.0, 0.0, -3.0 / 4.0 * M_PI, 0.0, 3.0 / 4.0 * M_PI, M_PI_4};

        // chiamo l'azione e aspetto che termini
        joint_client.sendGoalAndWait(joint_goal);

        // check dello stato dell'azione, se non ho errori lo stato deve essere SUCCEEDED
        if (joint_client.getState() != joint_client.getState().SUCCEEDED)
        {
            ROS_ERROR_STREAM("ERROR...");
            return -1;
        }

        // Attesa dell'utente
        ROS_INFO_STREAM("Traiettoria spazio giunti... OK - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;
    }

    // attivazione CLIK
    {
        // Attesa dell'utente
        ROS_INFO_STREAM("Attivo il CLIK... - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;

        // Attivo il CLIK chiamando il servizio di attivazione
        std_srvs::SetBool setBool_msg;
        setBool_msg.request.data = true; // <-- true = attiva
        bool success = set_run_clik_cli.call(setBool_msg);
        success = success && setBool_msg.response.success;
        if (!success) // check del successo del servizio
        {
            ROS_ERROR_STREAM("error " << setBool_msg.response.message);
            return -1;
        }

        // Attesa dell'utente
        ROS_INFO_STREAM("Attivo il CLIK... OK - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;
    }

    // goal cartesiano, lo dichiaro prima perchè lo uso per conservare pi e pf
    panda_clik_demo::CartesianTrajGoal cartesian_goal;

    // lettura pi (posa iniziale)
    {
        // Attesa dell'utente
        ROS_INFO_STREAM("Leggo la posa iniziale... - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;
        try
        {
            transformStamped = tfBuffer.lookupTransform("panda_link0", "panda_link8",
                                                        ros::Time(0), ros::Duration(5));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_ERROR_STREAM("TF ERROR " << ex.what());
            return -1;
        }

        // Copio la posa iniziale nel goal del Cartesian Traj
        cartesian_goal.duration = 3.0;
        cartesian_goal.pi.position.x = transformStamped.transform.translation.x;
        cartesian_goal.pi.position.y = transformStamped.transform.translation.y;
        cartesian_goal.pi.position.z = transformStamped.transform.translation.z;
        cartesian_goal.pi.orientation = transformStamped.transform.rotation;

        ROS_INFO_STREAM("La posa iniziale e' \n"
                        << cartesian_goal.pi);

        // Attesa dell'utente
        ROS_INFO_STREAM("Leggo la posa iniziale... OK - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;
    }

    // scelta di pf (posa finale)
    {
        // Attesa dell'utente
        ROS_INFO_STREAM("Setto la posa finale... - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;

        // La posa finale la scelgo a partire dalla posa iniziale dando un delta di 10cm lungo x e z, e 30cm lungo y.
        cartesian_goal.pf = cartesian_goal.pi;
        cartesian_goal.pf.position.z += 0.1;
        cartesian_goal.pf.position.x -= 0.1;
        cartesian_goal.pf.position.y += 0.3;

        ROS_INFO_STREAM("La posa finale e' \n"
                        << cartesian_goal.pf);

        // Attesa dell'utente
        ROS_INFO_STREAM("Setto la posa finale... OK - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;
    }

    // Movimento cartesiano
    {
        // Attesa dell'utente
        ROS_INFO_STREAM("Move Cartesian... - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;

        cartesian_goal.duration = 3.0;

        // chiamo l'azione e aspetto che termini
        cartesian_client.sendGoalAndWait(cartesian_goal);

        // check dello stato dell'azione, se non ho errori lo stato deve essere SUCCEEDED
        if (cartesian_client.getState() != cartesian_client.getState().SUCCEEDED)
        {
            ROS_ERROR_STREAM("ERROR...");
            return -1;
        }

        // Attesa dell'utente
        ROS_INFO_STREAM("Move Cartesian... OK - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;
    }

    // Stop del clik
    {
        // Attesa dell'utente
        ROS_INFO_STREAM("STOP del CLIK... - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;

        // Stoppo il CLIK chiamando il servizio di attivazione/disattivazione
        std_srvs::SetBool setBool_msg;
        setBool_msg.request.data = false; // <-- false = disattiva
        bool success = set_run_clik_cli.call(setBool_msg);
        success = success && setBool_msg.response.success;
        if (!success) // check del successo del servizio
        {
            ROS_ERROR_STREAM("error " << setBool_msg.response.message);
            return -1;
        }

        // Attesa dell'utente
        ROS_INFO_STREAM("STOP del CLIK... OK - Iserisci un carattere qualsiasi e premi invio:");
        std::cin >> ans;
    }

    // FINE

    return 0;
}
