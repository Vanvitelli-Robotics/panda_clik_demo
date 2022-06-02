#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "panda_clik_demo/JointTrajAction.h"
#include "sensor_msgs/JointState.h"

#include "panda_clik_demo/quintic_trajectory.h" // <--- quintic

/*
 Callback per le variabili di giunto
 Ogni volta che arriva un nuovo messaggio lo copio nella
 variabile globale 'joint_state' e setto la variabile
 globale booleana 'joint_state_arrived' a 'true'
*/
sensor_msgs::JointState joint_state;
bool joint_state_arrived = false;
void joint_states_cb(const sensor_msgs::JointState::ConstPtr msg)
{
    joint_state = *msg;
    joint_state_arrived = true;
}

/*
    Classe che mantiene il Joint Action server
*/
class JointTrajActionServer
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
    actionlib::SimpleActionServer<panda_clik_demo::JointTrajAction> as_;

    /*
    Elenco dei giunti da muovere.
    In questo esempio semplificato assumeremo di leggere le q0 già ordinate
    nell'ordine che ci aspettiamo (dal giunto 1 al giunto 7)
    */
    std::vector<std::string> joint_names = {"panda_joint1",
                                            "panda_joint2",
                                            "panda_joint3",
                                            "panda_joint4",
                                            "panda_joint5",
                                            "panda_joint6",
                                            "panda_joint7"};

    /*
        Costruttore con la lista di inizializzazione.
        Nota il boost::bind nella definizione della callbk!
    */
    JointTrajActionServer() : as_(nh_, "joint_traj_action", boost::bind(&JointTrajActionServer::executeCB, this, _1), false)
    {
    }

    /*
        Distruttore, non deve fare operazioni speciali.
    */
    ~JointTrajActionServer() = default;

    /*
        Metodo di start del server.
    */
    void start()
    {
        as_.start();
    }

    /*
        Execute callbk dell'action server
    */
    void executeCB(const panda_clik_demo::JointTrajGoalConstPtr &goal)
    {
        // In questo esempio ho scelto di costruire Publisher e Subscriber nella callbk.

        // Publisher per i comandi in posizione
        ros::Publisher joint_cmd_pub = nh_.advertise<sensor_msgs::JointState>("/cmd/joint_position", 1);

        /*
            Il seguente blocco serve per leggere la prima configurazione q0.

            Questa struttura è molto utile quando si vuole leggere in continuazione un messaggio in un ciclo.
            In generale la strategia è usare una variabile globale che è accessibile sia dalla callback del subscriber
            che dal ciclo stesso.

            In questo caso particolare vogliamo leggere un singolo messaggio: il blocco seguente può essere sostituito con
            ros::topic::waitForMessage<sensor_msgs::JointState>
        */
        // INIZIO BLOCCO PER LETTURA GIUNTI

        // inizializzo il subscriber
        ros::Subscriber joint_sub = nh_.subscribe("joint_states", 1, joint_states_cb);

        /*
            Ciclo di attesa del messaggio
            Metto il bool 'joint_state_arrived' a 'false'
            per essere sicuro di non prendermi un messaggio vecchio
        */
        joint_state_arrived = false;
        while (ros::ok() && !joint_state_arrived) // Finchè ros è ok e non è arrivato nessun messaggio
        {
            // controllo la preemprion (perchè siamo in una action)
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO_STREAM("Preempted");
                // set the action state to preempted
                as_.setPreempted();
                return; //<-- return termina la callbk dell'action
            }
            // nel ciclo faccio lo spinOnce che mi chiama la callback del subscriber (se il messaggio è arrivato)
            ros::spinOnce();
        }

        // alla fine copio la variabile globale 'joint_state' così da avere la configurazione iniziale
        // ed essere sicuro che sia costante
        // (nel seguito non uso la variabile globale 'joint_state' come configurazione iniziale perchè potrebbe cambiare a causa della callbk del subscriber)
        sensor_msgs::JointState qi = joint_state;
        // FINE BLOCCO PER LETTURA GIUNTI

        // stampa per visualizzarla
        ROS_INFO_STREAM("\nqi is:\n"
                        << qi);

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
        // Qui ho impostato 100 Hz
        ros::Rate loop_rate(100.0);

        /*
            Vero e proprio ciclo di "generazione traiettoria"
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

            // riempio il messaggio di comando
            sensor_msgs::JointState cmd;
            // cmd.position è un std::vector. Di default ha size=0. Devo fare un resize!
            cmd.position.resize(7);
            // riempio il vettore usando la funzione quintic che ho creato nella libreria "panda_clik_demo/quintic_trajectory.h"
            // qui ho supposto che qi è già ordinato correttamente.
            for (int i = 0; i < 7; i++)
                cmd.position[i] = qintic(t.toSec(), qi.position[i], goal->qf[i], goal->duration);

            // è importante riempire il vettore dei nomi dei giunti.
            cmd.name = joint_names;

            // riempio lo stamp, utile in caso di plot
            cmd.header.stamp = ros::Time::now();

            // ho definito come feedback il 'tempo che manca alla fine della traiettoria'
            // ovviamente non è l'unica scelta.
            panda_clik_demo::JointTrajFeedback feedbk_msg;
            feedbk_msg.time_left = ros::Duration(goal->duration) - t;

            // pubblico il feedback
            as_.publishFeedback(feedbk_msg);

            // publico il comando in giunti
            joint_cmd_pub.publish(cmd);
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
    ros::init(argc, argv, "joint_traj_action_server");

    // costruisco l'oggetto con il costruttore di default (nota che questo crea anche un NodeHandle)
    JointTrajActionServer server;

    // start dell'action server
    server.start();

    // spin per accettare le callbk dell'azione
    ros::spin();

    return 0;
}
