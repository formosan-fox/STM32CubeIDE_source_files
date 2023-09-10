/*
 * package.h
 *
 *  Created on: Aug 16, 2023
 *      Author: a9163
 */

#ifndef INC_PACKAGE_H_
#define INC_PACKAGE_H_

#include "platform.h"
#include <functional>

#define GET_STD_SIZE 10
#define START_DEPLOY_TIMER_SIZE 80
#define DEPLOY_TIMER_TARGET 65
#define LIFTOFF_SIZE 10
#define LIFTOFF_TARGET 9
#define SEPARATION_SIZE 20
#define SEPARATION_TARGET 18
#define GET_DOWNTREND_SIZE 50
#define DOWNTREND_TARGET 45

class Takeoff_separation_parachute_judgmental
{
public:
    int initial_altitude_number = 0;
    float initial_altitude_value;
    float std_altitude;
    float initial_altitude[10] = {0};
    int altitude_data_number = 0;
    float pre_WSEN_data = 0, cur_WSEN_data = 0;
    float YAXISACC_data = 0;
    int start_read_number = 0;


    int timer_altitude_count = 0; //for pressure data calculate
    int timer_yacc_count = 0; //for IMU data calculate
    float moving_avg_altitude = 0;
    float moving_avg_yacc = 0;
    float altitude_avg[5] = {0};
    float yacc_avg[5] = {0};

    // initial data function 
    void StartCorrectData(bool *DataReady, bool *breakLoop);
    void Initilize_altitude(bool *DataReady, float ALTITUDE, bool *breakLoop);
    float Initilize_getSTD();

    // determine whether the condition can be trigger or not
    void  StartDeployNumber(bool *DataReady, float yacc, bool *breakLoop);
    void  CheckTheRocketIsTakeoff(bool *DataReady, float altitude, bool *breakLoop);
    void  CheckTheRocketCanSeparation(bool *DataReady, float yacc, bool *breakLoop);
    void  CheckTheRocketCanDeploy(bool *DataReady, float altitude, bool *breakLoop, int DeployNumber);

    // timer call back function do
    void  Timer2GetAltitude(bool *DataReady, float *altitude);
    void  Timer2GetYacc(bool *DataReady, float *yacc);
    void  Timer2GetDeployTime(bool DeployReady, int *DeployNumber);

private:

};

template <class T, class U>
struct data {
    T comparing_value;
    U value;
};

template <class T, class U>
class Node {
   public:
    struct data<T, U> node_data;
    Node *next;
    // Constructor
    // create node and initlize data , next
    Node(T comparing_result, U value)  
    {
        this->node_data.comparing_value = comparing_result;
        this->node_data.value = value;
        this->next = nullptr;
    };
};

template <class T, class U>
class Queue {
   private:
    Node<T, U> *head;
    Node<T, U> *tail;
    int element_number;

   public:
    Queue() : head(nullptr), tail(nullptr), element_number(0) {}

    void push(T comparing_result, U value) 
    {
        Node<T, U> *new_node = new Node<T, U>(comparing_result, value);
        // one by one
        if (head == nullptr) {
            head = new_node;
            tail = new_node;
        } else {
            tail->next = new_node;
            tail = new_node;
        }
        element_number += 1;
    }

    void pop() {
        Node<T, U> *temp = head;

        head = head->next;
        delete temp;
        element_number -= 1;
    }

    struct data<T,U> get_head() {
        return head->node_data;
    }

    struct data<T,U> get_tail() {
        return tail->node_data;
    }

    int size() {
        return element_number;
    }
};


template <class T>
class EvaluationWindow {
   private:
    Queue<bool, T> window;     // a queue store success or fail
    int window_size;           // maximum element number of queue
    int success_number = 0;    // the number of success records
    bool (*evaluation)(T, T);  // evaluation function, depends on different situation
                               // function<int(int)> evaluation;

   public:
    // constructor
    EvaluationWindow(int _window_size, bool (*_evaluation)(T, T)) : window_size(_window_size), evaluation(_evaluation){};

    void updateData(T cur_data, T compared_data) 
    {
        // evaluate cur_data and compared_data to get comparing result
        bool comparing_result = evaluation(cur_data, compared_data);
        // update data into window. if result is true then add true into queue and make success_number increase 1.
        window.push(comparing_result, cur_data);
        if (comparing_result == true) {
            success_number += 1;
        }
        // check whether out of window_size
        if (window.size() > window_size) 
        {
            if (window.get_head().value == true) 
            {
                // if the one popped out is success,make success_number decrease 1.
                success_number -= 1;
            }
            window.pop();
        }
    }

    int getSuccessNumber() {
        return success_number;
    }
};



#endif /* INC_PACKAGE_H_ */
