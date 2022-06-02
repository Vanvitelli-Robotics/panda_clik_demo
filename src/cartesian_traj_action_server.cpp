#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "panda_clik_demo/CartesianTrajAction.h"
#include "geometry_msgs/PoseStamped.h"

#include "panda_clik_demo/quintic_trajectory.h" // <--- quintic

/*
    Classe che mantiene il Cartesian Action server
*/
class CartesianTrajActionServer
{

public:
    /*
        il node handle della rete ros
        ATTENZIONE! poichè nel main non abbiamo messo un nodehandle,
        e poichè almeno un nodehandle deve esistere nella rete ROS
        per inizializzarsi, è NECESSARIO che questo NodeHandle nh_ sia costruito
        _prima_ del SimpleActionServer as_. Quindi deve comparire prima di as_.
        Questo 'problema' è commentato anche nel codice tutorial di Fibonacci.
    */
    ros::NodeHandle nh_;

    // Istanza del SimpleActionServer
    actionlib::SimpleActionServer<panda_clik_demo::CartesianTrajAction> as_;

    // In questo esempio ho scelto di dichiarare il publiscer fuori dalla callbk (a differenza del JointActionServer)
    // l'inizializzazione del publisher viene fatta nel metodo start()
    // (potevo farla anche nel costruttore)
    ros::Publisher cartesian_pub;

    /*
        Costruttore con la lista di inizializzazione.
        Nota il boost::bind nella definizione della callbk!
    */
    CartesianTrajActionServer() : as_(nh_, "cartesian_traj_action", boost::bind(&CartesianTrajActionServer::executeCB, this, _1), false)
    {
    }

    /*
        Distruttore, non deve fare operazioni speciali.
    */
    ~CartesianTrajActionServer() = default;

    /*
        Metodo di start del server.
        Inizializzo anche il publisher qui
    */
    void start()
    {
        // il publisher andrà a pubblicare su un topic intermedio letto dal nodo di inversione cinematica CLIK
        cartesian_pub = nh_.advertise<geometry_msgs::PoseStamped>("/clik/desired_pose", 1);
        as_.start();
    }

    /*
        Execute callbk dell'action server
    */
    void executeCB(const panda_clik_demo::CartesianTrajGoalConstPtr &goal)
    {
        /*
            In questo generatore non ho bisogno della posa iniziale perchè mi viene data direttamente dal goal
        */
        /*
            Gestione del tempo ROS
            Nel ciclo mi servirà il tempo dall'inizio della traiettoria t.
            Uso ros::Time::now() che resistuisce il timestamp del PC (secondi trascorsi da 1/1/1970)
            Per avere t, mi salvo il tempo macchina dell'inizio della traiettoria t0.
            Poi t satà ros::Time::now()-t0
            Curiosità: gli oggetti Time e Duration in ROS hanno le definizioni degli operatori matematici
            TIME - TIME = DURATION
            http://wiki.ros.org/roscpp/Overview/Time
        */
        ros::Time t0 = ros::Time::now();
        ros::Duration t = ros::Time::now() - t0;

        // La nostra traiettoria sarà pubblicata con una certa frequenza.
        // Qui ho impostato 1000 Hz
        ros::Rate loop_rate(1000.0);

        /*
            Ciclo di "generazione traiettoria"
            cicla finchè ros è ok e non ho finito la traiettoria
            t.toSec() <= goal->duration significa t<tf.
        */
        while (ros::ok() && t.toSec() <= goal->duration)
        {
            // sleep sul rate
            loop_rate.sleep();

            // calcolo il nuovo t
            t = ros::Time::now() - t0;

            // check preemprtion dell'azione
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO_STREAM("Preempted");
                // set the action state to preempted
                as_.setPreempted();
                return;
            }

            // riempio il messaggio di comando usando la funzione quintic che ho creato nella libreria "panda_clik_demo/quintic_trajectory.h"
            geometry_msgs::PoseStamped out_msg;
            out_msg.pose.position.x = qintic(t.toSec(), goal->pi.position.x, goal->pf.position.x, goal->duration);
            out_msg.pose.position.y = qintic(t.toSec(), goal->pi.position.y, goal->pf.position.y, goal->duration);
            out_msg.pose.position.z = qintic(t.toSec(), goal->pi.position.z, goal->pf.position.z, goal->duration);
            // l'orientamento è costante e lo pongo uguale a pf.
            // Nota che, a causa dei limiti di questo generatore (genera solo una traiettoria in posizione e non in orientamento)
            // l'orientamento iniziale _DEVE_ essere lo stesso di qello finale. In questo esempio non faccio nessun controllo
            // ma sarebbe il caso di controllare che pf.orientation == pi.orientation e generare un eccezione in caso negativo.
            out_msg.pose.orientation = goal->pf.orientation;

            // sto usando la versione Stamped del messaggio posa. Sarebbe opportuno riempire l'header.stamp (il tempo attuale del messaggio)
            out_msg.header.stamp = ros::Time::now();

            // ho definito come feedback il 'tempo che manca alla fine della traiettoria'
            // ovviamente non è l'unica scelta.
            panda_clik_demo::CartesianTrajFeedback feedbk_msg;
            feedbk_msg.time_left = ros::Duration(goal->duration) - t;

            // pubblico il feedback
            as_.publishFeedback(feedbk_msg);

            // publico il comando in cartesiano
            cartesian_pub.publish(out_msg);
        }

        // se sono arrivato fin qui, non ci sono stati errori o preeption e la traiettoria è finita
        as_.setSucceeded();
    }
};

/*
    Il main
*/
int main(int argc, char *argv[])
{
    // init
    ros::init(argc, argv, "cartesian_traj_action_server");

    // costruisco l'oggetto con il costruttore di default (nota che questo crea anche un NodeHandle)
    CartesianTrajActionServer server;

    // start dell'action server
    server.start();

    // spin per accettare le callbk dell'azione
    ros::spin();

    return 0;
}
