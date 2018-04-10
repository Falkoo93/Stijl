#include "../header/functions.h"

char mode_start;

void write_in_queue(RT_QUEUE *, MessageToMon);


//------------------------------------------------------------------------------
void f_niveauBatterie (void *arg)
{
	int tensionBatterie;
	MessageToMon msg;
 
	RT_TASK_INFO info;
	rt_task_inquire(NULL, &info);
	printf("Init %s\n", info.name);
	rt_sem_p(&sem_barrier, TM_INFINITE);
	/* PERIODIC START */
    #ifdef _WITH_TRACE_
   	 printf("%s: start period\n", info.name);
    #endif
   	 rt_task_set_periodic(NULL, TM_NOW, 5000000);
	while(1){
   	 rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
   	 	if(robotStarted == 1){
   	 rt_mutex_release(&mutex_robotStarted);
   	 tensionBatterie=send_command_to_robot(DMB_GET_VBAT);
  		 set_msgToMon_header(&msg, HEADER_STM_BAT);
   		 write_in_queue(&q_messageToMon, msg);
		}
	}
}
//------------------------------------------------------------------------------

void f_perte_info(void *arg){
	RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
	while(1){
		rt_mutex_acquire(&mutex_etat_communication, TM_INFINITE);
		if(etat_communication == 1) {
			rt_mutex_release(&mutex_etat_communication);
		} else {
			rt_mutex_release(&mutex_etat_communication);
			MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
			kill_nodejs();
			close_server();
			printf("Nodejs is lost\n");
		}		
	}	
}

void f_server(void *arg) {
    int err;
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    err = run_nodejs("/usr/local/bin/node", "/home/pi/Interface_Robot/server.js");

    if (err < 0) {
        printf("Failed to start nodejs: %s\n", strerror(-err));
        exit(EXIT_FAILURE);
    } else {
#ifdef _WITH_TRACE_
        printf("%s: nodejs started\n", info.name);
#endif
        open_server();
        rt_sem_broadcast(&sem_serverOk);
    }
}

void f_sendToMon(void * arg) {
    int err;
    MessageToMon msg;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
#ifdef _WITH_TRACE_
    printf("Changing Etat Communication to 1\n");
#endif
	rt_mutex_acquire(&mutex_etat_communication, TM_INFINITE);
    etat_communication = 1;
    rt_mutex_release(&mutex_etat_communication);
    while (1) {

#ifdef _WITH_TRACE_
        printf("%s : waiting for a message in queue\n", info.name);
#endif
        if (rt_queue_read(&q_messageToMon, &msg, sizeof (MessageToRobot), TM_INFINITE) >= 0) {
#ifdef _WITH_TRACE_
            printf("%s : message {%s,%s} in queue\n", info.name, msg.header, msg.data);
#endif

            send_message_to_monitor(msg.header, msg.data);
            free_msgToMon_data(&msg);
            rt_queue_free(&q_messageToMon, &msg);
        } else {
            printf("Error msg queue write: %s\n", strerror(-err));
        }
    }
}

void f_receiveFromMon(void *arg) {
    MessageFromMon msg;
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

#ifdef _WITH_TRACE_
    printf("%s : waiting for sem_serverOk\n", info.name);
#endif
    rt_sem_p(&sem_serverOk, TM_INFINITE);
#ifdef _WITH_TRACE_
    printf("Changing Etat Communication to 1\n");
#endif 
    rt_mutex_acquire(&mutex_etat_communication, TM_INFINITE);
    etat_communication = 1;
    rt_mutex_release(&mutex_etat_communication);
    do {
#ifdef _WITH_TRACE_
        printf("%s : waiting for a message from monitor\n", info.name);
#endif
        
        err = receive_message_from_monitor(msg.header, msg.data);
#ifdef _WITH_TRACE_
        printf("%s: msg {header:%s,data=%s} received from UI\n", info.name, msg.header, msg.data);
#endif
        if (strcmp(msg.header, HEADER_MTS_COM_DMB) == 0) {
            if (msg.data[0] == OPEN_COM_DMB) { // Open communication supervisor-robot
#ifdef _WITH_TRACE_
                printf("%s: message open Xbee communication\n", info.name);
#endif
                rt_sem_v(&sem_openComRobot);
            }
        } else if (strcmp(msg.header, HEADER_MTS_DMB_ORDER) == 0) {
            if ((msg.data[0] == DMB_START_WITHOUT_WD)||(msg.data[0] == DMB_START_WITH_WD)) { // Start robot
#ifdef _WITH_TRACE_
                printf("%s: message start robot\n", info.name);
#endif 
                rt_sem_v(&sem_startRobot);

            } else if ((msg.data[0] == DMB_GO_BACK)
                    || (msg.data[0] == DMB_GO_FORWARD)
                    || (msg.data[0] == DMB_GO_LEFT)
                    || (msg.data[0] == DMB_GO_RIGHT)
                    || (msg.data[0] == DMB_STOP_MOVE)) {

                rt_mutex_acquire(&mutex_move, TM_INFINITE);
                move = msg.data[0];
                rt_mutex_release(&mutex_move);
#ifdef _WITH_TRACE_
                printf("%s: message update movement with %c\n", info.name, move);
#endif

            }
        } else if (strcmp(msg.header, HEADER_MTS_CAMERA) == 0) {
            if (msg.data[0] == CAM_OPEN) { 
                rt_sem_v(&sem_connexionCamera);
            } else if (msg.data[0] == CAM_ASK_ARENA) {
                rt_sem_v(&sem_arena);
            } else if (msg.data[0] == CAM_COMPUTE_POSITION) {
                rt_sem_v(&sem_position);
            }
        }    
        
    } while (err > 0);

}

void f_openComRobot(void * arg) {
    int err;

    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_openComRobot\n", info.name);
#endif
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_openComRobot arrived => open communication robot\n", info.name);
#endif
        err = open_communication_robot();
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the communication is opened\n", info.name);
#endif
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }
}

void f_startRobot(void * arg) {
    int err;
    
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    while (1) {
#ifdef _WITH_TRACE_
        printf("%s : Wait sem_startRobot\n", info.name);
#endif
        rt_sem_p(&sem_startRobot, TM_INFINITE);
#ifdef _WITH_TRACE_
        printf("%s : sem_startRobot arrived => Start robot\n", info.name);
#endif
        err= send_command_to_robot(cmd);
         if (strcmp(cmd, DMB_START_WITHOUT_WD) == 0) {
        //err = send_command_to_robot(DMB_START_WITHOUT_WD);
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the robot is started\n", info.name);
#endif
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }

     else if (strcmp(cmd, DMB_START_WITH_WD) == 0) {
        //err = send_command_to_robot(DMB_START_WITH_WD);
        if (err == 0) {
#ifdef _WITH_TRACE_
            printf("%s : the robot is started\n", info.name);
#endif
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            robotStarted = 1;
            rt_mutex_release(&mutex_robotStarted);
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_ACK);
            write_in_queue(&q_messageToMon, msg);
            rt_sem_v(&sem_watchdog);

        } else {
            MessageToMon msg;
            set_msgToMon_header(&msg, HEADER_STM_NO_ACK);
            write_in_queue(&q_messageToMon, msg);
        }
    }


    }
}


void f_watchdog(void*arg){
     RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_watchdog, TM_INFINITE);

      /* PERIODIC START */
#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
     while (1) {
#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif
        rt_task_wait_period(NULL);
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted) {
            rt_mutex_acquire(&Cmpt, TM_INFINITE){
            if (Cmpt<3){
                Cmpt=Cmpt+1;
                send_command_to_robot(DMB_RELOAD_WD);
                
            }
            else {
                send_command_to_robot(DMB_RELOAD_WD);
            }
            
            rt_mutex_release(&mutex_Cmpt);
            }
        rt_mutex_release(&mutex_robotStarted);
    }
    }
}

void f_move(void *arg) {
    /* INIT */
    RT_TASK_INFO info;
    rt_task_inquire(NULL, &info);
    printf("Init %s\n", info.name);
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /* PERIODIC START */
#ifdef _WITH_TRACE_
    printf("%s: start period\n", info.name);
#endif
    rt_task_set_periodic(NULL, TM_NOW, 100000000);
    while (1) {
#ifdef _WITH_TRACE_
        printf("%s: Wait period \n", info.name);
#endif
        rt_task_wait_period(NULL);
#ifdef _WITH_TRACE_
        printf("%s: Periodic activation\n", info.name);
        printf("%s: move equals %c\n", info.name, move);
#endif
        rt_mutex_acquire(&mutex_Cmpt, TM_INFINITE);
        if (Cmpt<3) {
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        if (robotStarted) {

            
#ifdef _WITH_TRACE_
        printf("%s : sem_openComRobot arrived => open communication robot\n", info.name);
#endif
         err = open_communication_robot();
        if (err == 0) {
        send_command_to_robot(DMB_RELOAD_WD);
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            send_command_to_robot(move);
            rt_mutex_release(&mutex_move);
        }
        rt_mutex_release(&mutex_robotStarted);
#ifdef _WITH_TRACE_
            printf("%s: the movement %c was sent\n", info.name, move);
#endif            
        }
        else{
            rt_sem_p(&sem_openComRobot, TM_INFINITE);
            close_comunication_robot();
            MessageToMon msg;
            set_msgToMon_header(&msg, CLOSE_COM_DMB);
            write_in_queue(&q_messageToMon, msg);

        }
        rt_mutex_release(&mutex_Cmpt);
    }
}
}

void f_camera(void * arg)                         //RajoutéJ
{
    	int err;
    	/* INIT */
    	RT_TASK_INFO info;
    	rt_task_inquire(NULL, &info);
    	printf("Init %s\n", info.name);
    	rt_sem_p(&sem_barrier, TM_INFINITE);
    
    	rt_sem_p(&sem_connexionCamera, TM_INFINITE);
    
    	opencamera(Camera);
    
    	rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        cameraStarted=1;
    	rt_mutex_release(&mutex_cameraStarted);
}

void f_image(void * arg)                         //RajoutéJ
{
    	int err;
	MessageToMon msg;
	int nbrTriangle=0
		
    	/* INIT */
    	RT_TASK_INFO info;
    	rt_task_inquire(NULL, &info);
    	printf("Init %s\n", info.name);
    	rt_sem_p(&sem_barrier, TM_INFINITE);
	
	/* PERIODIC START */
	#ifdef _WITH_TRACE_
    	printf("%s: start period\n", info.name);
	#endif
    	rt_task_set_periodic(NULL, TM_NOW, 1000000); //temps en tic d'horlogo = ns???????
	
	if(cameraStarted==1)
	{
		if(calculPosition==1)
		{
			nbrtriangle=detect_position(imgInput, posTriangle, monArene);
			if(nbrtriangle==1)
			{
				draw_position(imgInput,imgOutput,positionRobot); 
				set_msgToMon_header(&msg, HEADER_STM_POS);
			}
			else
			{
				posrobot=(-1,-1)///////////////A CHANGER!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
				set_msgToMon_header(&msg, HEADER_STM_POS);
			}
		}
		else if(calculPosition==0)
		{
			get_image(camera,monImage,fichier);
			if(demandeArene==0)
			{
				compress_image(imgInput,imageCompress);
				set_msgToMon_header(&msg, HEADER_STM_IMAGE);
			}
			else if(demandeArene==1)
			{
				err=detect_arena(Image *monImage, Arene *rectangle)
				if(err=0)
				{
					draw_arena(imgInput,imgOutput,monArene);
					compress_image(imgInput,imageCompress);
					set_msgToMon_header(&msg, HEADER_STM_IMAGE);
				}
				else if(err=-1)
				{
					compress_image(imgInput,imageCompress);
					set_msgToMon_header(&msg, HEADER_STM_IMAGE);
				}
				
				///////////////////A CHANGER//////////////////////////////
				if(validation_utilisateur???????)
				{
					save(????????????????????????)
					rt_mutex_acquire(&mutex_cameraStarted, TM_INFINITE);
        				demandeArene=0;//pour repasser dans le mode d'envoie normal périodique d'image
    					rt_mutex_release(&mutex_cameraStarted);
				}
				else if(pas validation_utilisateur???????)
				{
					delete(????????????????????????)
				}
				//////////////////////////////////////////////////////////
					
			}
		}
		
		
	}
	
	
    
}


void write_in_queue(RT_QUEUE *queue, MessageToMon msg) {
    void *buff;
    buff = rt_queue_alloc(&q_messageToMon, sizeof (MessageToMon));
    memcpy(buff, &msg, sizeof (MessageToMon));
    rt_queue_send(&q_messageToMon, buff, sizeof (MessageToMon), Q_NORMAL);
}
