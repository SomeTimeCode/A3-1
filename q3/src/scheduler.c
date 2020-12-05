#include "../include/scheduler.h"

void *simpleRobotRoutine(void *arg) {
  Robot robot = (Robot)arg;
  Task task = robot->task;
#ifdef DEBUG
    debug_printf(__func__, "Robot%c[%d] starts...\n", 
        RobotTypeToChar(robot->robotType), robot->id);
#endif 
  int jobID;
  //the job incoming in the queue
  int front;
  //the number of robots are free in the time with respectively type
  int a;
  int b;
  int c;
  //different robots have different preference to choose to job
  if(robot->robotType == TypeA){
    while (!queueIsEmpty(task->jobQ)) {
      sem_wait(&task->pick);
      b = task->typeB;
      c = task->typeC;
      if(queueIsEmpty(task->jobQ)){
        sem_post(&task->pick);
        break;
      }
      front= *queueFront(task->jobQ);
      if((front == 0|| front == 1 || front == 2) && task->bodyIsHere < 1){
        sem_post(&task->pick);
        continue;
      }
      if((front== 0 || front== 1|| front==5 || front==7) && c != 0){
        sem_post(&task->pick);
        continue;
      }
      if(front== 6 && b != 0){
        sem_post(&task->pick);
        continue;
      }
      //reduce the number of robots available if assigned a job to do
      task->typeA = task->typeA - 1;
      queueDequeueFront(task->jobQ, &jobID);
      sem_post(&task->pick);
  #ifdef DEBUG
      debug_printf(__func__, "Robot%c[%d]: working on job %d...\n", 
          RobotTypeToChar(robot->robotType), robot->id, jobID);
  #endif
      simpleWork(jobID, robot);
      task->typeA = task->typeA + 1;
    }
    pthread_exit(NULL);
  }else if(robot->robotType == TypeB){
    while (!queueIsEmpty(task->jobQ)) {
      sem_wait(&task->pick);
      a = task->typeA;
      c = task->typeC;
      if(queueIsEmpty(task->jobQ)){
        sem_post(&task->pick);
        break;
      }
      front= *queueFront(task->jobQ);
      if((front == 0|| front == 1 || front == 2) && task->bodyIsHere < 1){
        sem_post(&task->pick);
        continue;
      }
      if((front== 0 || front== 1|| front==5 || front==7) && c != 0){
        sem_post(&task->pick);
        continue;
      }
      if((front==2||front==3||front==4) && (a!= 0)){
        sem_post(&task->pick);
        continue;
      }
      task->typeB = task->typeB - 1;
      queueDequeueFront(task->jobQ, &jobID);
      sem_post(&task->pick);
  #ifdef DEBUG
      debug_printf(__func__, "Robot%c[%d]: working on job %d...\n", 
          RobotTypeToChar(robot->robotType), robot->id, jobID);
  #endif
      simpleWork(jobID, robot);
      task->typeB = task->typeB + 1;
    }
    pthread_exit(NULL);
  }else{
    while (!queueIsEmpty(task->jobQ)) {
      sem_wait(&task->pick);
      a = task->typeA;
      b = task->typeB;
      if(queueIsEmpty(task->jobQ)){
        sem_post(&task->pick);
        break;
      }
      front= *queueFront(task->jobQ);
      if((front == 0|| front == 1 || front == 2) && task->bodyIsHere < 1){
        sem_post(&task->pick);
        continue;
      }
      if((front== 6) && b != 0){
        sem_post(&task->pick);
        continue;
      }
      if((front==2||front==3||front==4) && (a!= 0)){
        sem_post(&task->pick);
        continue;
      }
      task->typeC = task->typeC - 1;
      queueDequeueFront(task->jobQ, &jobID);
      sem_post(&task->pick);
  #ifdef DEBUG
      debug_printf(__func__, "Robot%c[%d]: working on job %d...\n", 
          RobotTypeToChar(robot->robotType), robot->id, jobID);
  #endif
      simpleWork(jobID, robot);
      task->typeC = task->typeC + 1;
    }
    pthread_exit(NULL);
  }
}






















void simpleWork(int jobID, Robot robot) {
  Task task = robot->task;
  double timer = getCurrentTime();
  switch (jobID) {
  case SKELETON:
#ifdef DEBUG
    debug_printf(__func__, "Robot%c[%d] making skeleton...\n", 
        RobotTypeToChar(robot->robotType), robot->id);
    fflush(stdout);
#endif
    makeSkeleton(robot);
    break;
  case ENGINE:
#ifdef DEBUG
    debug_printf(__func__, "Robot%c[%d] making engine...\n", 
        RobotTypeToChar(robot->robotType), robot->id);
    fflush(stdout);
#endif
    makeEngine(robot);
    break;
  case CHASSIS:
#ifdef DEBUG
    debug_printf(__func__, "Robot%c[%d] making chassis...\n", 
        RobotTypeToChar(robot->robotType), robot->id);
    fflush(stdout);
#endif
    makeChassis(robot);
    break;
  case BODY:
    task->bodyIsHere = task->bodyIsHere + 1;
    makeBody(robot);
    task->bodyIsHere = task->bodyIsHere - 1;
    break;
  case WINDOW:
    makeWindow(robot);
    break;
  case TIRE:
    makeTire(robot);
    break;
  case BATTERY:
    makeBattery(robot);
    break;
  case CAR:
    makeCar(robot);
    break;
  default:
    err_printf(__func__, __LINE__, "Error!! Robot%c[%d] gets invalid jobID %d\n", 
        RobotTypeToChar(robot->robotType), robot->id, jobID);
    break;
  }
  timer = getCurrentTime() - timer;
#ifdef DEBUG
  debug_printf(__func__, "Robot%c[%d] job %d done! Time: %lf\n", 
      RobotTypeToChar(robot->robotType), robot->id, jobID, timer);
#endif
}
