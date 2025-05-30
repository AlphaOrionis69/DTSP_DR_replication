#include "scip/scip.h"
#include "scip/scipdefplugins.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include <string.h>
#include <limits.h>
#include <errno.h>
#include <time.h>



#define TRUCK_SPEED 1
#define DRONE_SPEED 2
#define DRONE_CAPACITY 4
#define DRONE_FLIGHT_ENDURANCE 200
double H;
#define M 5000
#define RESUPPLY_TIME 0
#define NUMBER_OF_DRONES 2
#define MAXPOINTS 105

typedef struct Points {
    double x;
    double y;
    int id;
    double releaseTime;
} Points;

double truckTravelTime[MAXPOINTS+2][MAXPOINTS+2];
double droneTravelTime[MAXPOINTS+2];






// helper to solve TSP faster
#include "TSP_heuristic.h"
// helper to solve the perfect-information scenario
#include "PI.h"
int min(int a, int b) {
    return a > b ? b : a;
}
void delay(int number_of_seconds) {
	// Converting time into milli_seconds
	int milli_seconds = 1000 * number_of_seconds;

	// Storing start time
	clock_t start_time = clock();

	// looping till required time is not achieved
	while (clock() < start_time + milli_seconds)
		;
}



// N: the set of orders
// E: the set of edges (i, j)
// K: the set of drones


double cal_dist(Points a, Points b) {
    return sqrt(((a.x)-(b.x))*((a.x)-(b.x)) + ((a.y)-(b.y))*((a.y)-(b.y)));
}

typedef struct pos {
    double x; double y;
} pos;

typedef struct TRUCK {
    int* loaded; // loaded order on the truck
    int* onboard; // onboard order on the drone
    int* available; // available order at the depot

    int* route; // the current route of the truck
    double* startTime; // time truck start from a node
    int* visited; // node visited by the truck

    // track the last element of the route
    int top_route;
    pos curpos; // the current position of the truck
    // other arr can be solved by marking 1 or 0 depending on current state
} TRUCK;
TRUCK Truck;

void initTruck(Points* listofPoints, int numberofPoints) {
    Truck.available = (int*)calloc(numberofPoints+2, sizeof(int)); // use index from 1 to N for clarity
    Truck.onboard = (int*)calloc(numberofPoints+2, sizeof(int));
    Truck.loaded = (int*)calloc(numberofPoints+2, sizeof(int));
    Truck.route = (int*)calloc(numberofPoints+2, sizeof(int));
    Truck.startTime = (double*)calloc(numberofPoints+2, sizeof(double));
    Truck.visited = (int*)calloc(numberofPoints+2, sizeof(int));
    Truck.top_route = -1;

    // unused coordinates
    Truck.curpos.x = listofPoints[0].x;
    Truck.curpos.y = listofPoints[0].y;

    // all orders with release time equals to 0 is marked as loaded and visited
    // since truck has unlimited capacity, and all orders at the beginning needed to be satisfied
    for (int i = 1; i <= numberofPoints; i++) {
        if (listofPoints[i].releaseTime == 0) {
            Truck.loaded[i] = 1;
            Truck.visited[i] = 1;
        }
    }
    Truck.visited[0] = 1; // depot always be visited
}

typedef struct SORTIE {
    double launchTime; // launch time of the sortie
    int meetingLocation; // the location where drone meet the truck (based on id)
    int* setofOrders; // the list of order this sortie would resupply
    int topOrder;
} SORTIE;

typedef struct DRONE {
    double availableTime; // time drone can be available at depot
    int* onboard; // onboard order on the drone
    int top_onboard;
    SORTIE* listofSortie;
    int topSortie;

    pos curpos; // store the current position of the drone

    SORTIE lastPerformedSortie; // store the sortie that drone is performing
} DRONE;
DRONE Drone[NUMBER_OF_DRONES];

void initAllDrone(Points depot, int numberofPoints) {
    for (int i = 0; i < NUMBER_OF_DRONES; i++) {
        Drone[i].availableTime = 0.0;
        Drone[i].onboard = (int*)calloc(DRONE_CAPACITY, sizeof(int)); // the maximum number of orders onboard is Q
        Drone[i].top_onboard = -1;
        Drone[i].listofSortie = (SORTIE*)malloc(sizeof(SORTIE)*((numberofPoints+2)/DRONE_CAPACITY+1));
        Drone[i].topSortie = -1;

        // unused coordinates
        Drone[i].curpos.x = depot.x;
        Drone[i].curpos.y = depot.y;

        SORTIE holder = {-1, -1.0, NULL, -1};
        Drone[i].lastPerformedSortie = holder;
    }
}


// calculate the traveling time of truck and drone
void initTravelTime(Points* listofPoints, int numberofPoints) {
    for (int i = 0; i < numberofPoints+2; i++) {
        for (int j = 0; j < numberofPoints+2; j++) {
            if (i == j) continue;
            truckTravelTime[i][j] = (cal_dist(listofPoints[i], listofPoints[j]))/(1.0*TRUCK_SPEED);
        }
    }
    for (int i = 1; i <= numberofPoints; i++) {
        droneTravelTime[i] = (cal_dist(listofPoints[0], listofPoints[i]))/(1.0*DRONE_SPEED);
    }
}



// initialize a TSP tour for the available orders at the beginning of the day
void initTruckRoute(Points* listofPoints, int numberofPoints, double* CompletionTime) {
    // generate TSP tour using MILP (slow)
    {
        /*
        SCIP* scip;
        // create a TSP problem
        SCIP_CALL(SCIPcreate(&scip));
        SCIP_CALL(SCIPincludeDefaultPlugins(scip));
        SCIP_CALL(SCIPcreateProbBasic(scip, "TSP"));

        SCIP_VAR* x[numberofPoints+2][numberofPoints+2];
        SCIP_VAR* T[numberofPoints+2];

        // x[i][j] = 1 if the route include traveling from node i to node j
        // T[i] denote the start time of the truck at node i
        char name[64];
        // objective function: min T[numberofPoints+1] || truck_time[i][j]*x[i][j]

        // x[i][j]
        for (int i = 0; i <= numberofPoints; i++) {
            for (int j = 1; j <= numberofPoints+1; j++) {
                if (i == j) continue;
                if (i == 0 && j == numberofPoints+1) continue;
                snprintf(name, sizeof(name), "x_%d_%d", i, j);
                double objcoef = truckTravelTime[listofPoints[i].id][listofPoints[j].id];
                SCIP_CALL(SCIPcreateVarBasic(scip, &x[i][j], name, 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY));
                SCIP_CALL(SCIPaddVar(scip, x[i][j]));
            }
        }

        // T[i]
        for (int i = 0; i <= numberofPoints+1; i++) {
            snprintf(name, sizeof(name), "T_%d", i);
            double objcoef = (i == numberofPoints+1) ? 1.0 : 0.0;
            SCIP_CALL(SCIPcreateVarBasic(scip, &T[i], name, 0.0, SCIPinfinity(scip), objcoef, SCIP_VARTYPE_CONTINUOUS));
            SCIP_CALL(SCIPaddVar(scip, T[i]));
        }

        SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));
        // adding constraints
        SCIP_VAR* vars[numberofPoints+2];
        double coefs[numberofPoints+2];
        SCIP_CONS* cons;
        int cnt;
        for (int i = 0; i <= numberofPoints; i++) {
            cnt = 0;
            for (int j = 1; j <= numberofPoints+1; j++) {
                if (i == j || (i == 0 && j == numberofPoints+1)) continue;
                vars[cnt] = x[i][j]; coefs[cnt] = 1.0; cnt++;
            }
            snprintf(name, sizeof(name), "c1_%d", i);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 1.0, 1.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
        for (int j = 1; j <= numberofPoints+1; j++) {
            cnt = 0;
            for (int i = 0; i <= numberofPoints; i++) {
                if (i == j || (i == 0 && j == numberofPoints+1)) continue;
                vars[cnt] = x[i][j]; coefs[cnt] = 1.0; cnt++;
            }
            snprintf(name, sizeof(name), "c2_%d", j);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 1.0, 1.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
        // T[i] >= T[j] + truck_time[j][i] - M(1 - x[j][i]) for i in N || i = n+1, j in N || j = 0; i != j || (i == 0 & j == n+1) = 0
        {
            for (int i = 1; i <= numberofPoints+1; i++) {
                for (int j = 0; j <= numberofPoints; j++) {
                    if (i == j || (j == 0 && i == numberofPoints+1)) continue;
                    cnt = 0;
                    vars[cnt] = T[i]; coefs[cnt] = 1.0; cnt++;
                    vars[cnt] = T[j]; coefs[cnt] = -1.0; cnt++;
                    vars[cnt] = x[j][i]; coefs[cnt] = -1.0*M; cnt++;
                    double lhs = truckTravelTime[listofPoints[j].id][listofPoints[i].id] - M;
                    snprintf(name, sizeof(name), "c5_%d_%d", i, j);
                    SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, lhs, SCIPinfinity(scip)));
                    SCIP_CALL(SCIPaddCons(scip, cons));
                    SCIPreleaseCons(scip, &cons);
                }
            }
        }

        // solving:
        SCIP_CALL(SCIPsolve(scip));
        SCIP_SOL* sol = SCIPgetBestSol(scip);
        if (sol) {
            // add the route found into truck
            // assume the end node is always the depot
            Truck.top_route = numberofPoints+1;
            int id = Truck.top_route;
            int curPoint = numberofPoints+1;
            while (curPoint != 0) {

                printf("curPoint: %d\n", curPoint);

                Truck.route[id] = listofPoints[curPoint].id;
                for (int i = 0; i <= numberofPoints; i++) {
                    if (i == curPoint || (i == 0 && curPoint == numberofPoints+1)) continue;
                    if (SCIPgetSolVal(scip, sol, x[i][curPoint]) > 0.5) {
                        curPoint = i;
                        id--; break;
                    }
                }
            }
            Truck.route[id] = listofPoints[0].id;
            printf("Total distance: %f\n", SCIPgetSolOrigObj(scip, sol));
            *CompletionTime = SCIPgetSolOrigObj(scip, sol); // save the solution value to completionTime

            for (int i = 0; i <= numberofPoints; i++) {
                for (int j = 1; j <= numberofPoints+1; j++) {
                    if (i == j || (i == 0 && j == numberofPoints+1)) continue;
                    if (SCIPgetSolVal(scip, sol, x[i][j]) > 0.5) {
                        printf("%d %d\n", i, j);
                    }
                }
            }

        }
        for (int i = 0; i <= Truck.top_route; i++) {
            printf("%d ", Truck.route[i]);
        }
        printf("\n");
        // calculate the startTime of the truck at each node in the route
        // there is no waiting time -> accumulating it
        double curTime = 0;
        for (int i = 0; i <= Truck.top_route; i++) {
            Truck.startTime[i] = curTime;
            printf("%lf ", curTime);
            if (i == Truck.top_route) break;
            curTime += truckTravelTime[Truck.route[i]][Truck.route[i+1]];
        }
        printf("\n");
        for (int i = 0; i <= Truck.top_route; i++) {
            printf("%lf %lf\n", Truck.startTime[i], SCIPgetSolVal(scip, sol, T[i]));
        }


        // release var
        for (int i = 0; i <= numberofPoints; i++) {
            for (int j = 1; j <= numberofPoints+1; j++) {
                if (i == j || (i == 0 && j == numberofPoints+1)) continue;
                SCIP_CALL(SCIPreleaseVar(scip, &x[i][j]));
            }
        }
        for (int i = 0; i <= numberofPoints+1; i++) {
            SCIP_CALL(SCIPreleaseVar(scip, &T[i]));
        }
        //SCIP_CALL(SCIPreleaseCons(scip, &cons));
        SCIP_CALL(SCIPfree(&scip));
        */
    }

    // prepare input
    // create travelTime 2d-arr to map the distance
    {
        int total = numberofPoints+2;
        double** travelTime = (double**)malloc(sizeof(double*)*total);
        for (int i = 0; i < total; i++) {
            travelTime[i] = (double*)malloc(sizeof(double)*total);
            // ex: listofPoints: 0, 1, 7, 8, 0, but when transfer it turns to be 0, 1, 2, 3, 4
            for (int j = 0; j < total; j++) {
                travelTime[i][j] = truckTravelTime[listofPoints[i].id][listofPoints[j].id];
            }
        }

        int* final_tour = (int*)malloc(sizeof(int)*total); // final tour would contain 0, 1, 2, 3, 4
        printf("DONE\n");
        TSP_heuristic(travelTime, total, final_tour);

        // now copy the final tour to the truck route;
        for (int i = 0; i < total; i++) {
            Truck.route[i] = listofPoints[final_tour[i]].id;
        }
        //update startTime by accumulate it
        double curTime = 0.0;
        for (int i = 0; i < total; i++) {
            Truck.startTime[i] = curTime;
            if (i == total-1) break;
            curTime += truckTravelTime[Truck.route[i]][Truck.route[i+1]];
        }
        for (int i = 0; i < total; i++) {
            printf("%d ", Truck.route[i]);
        }
        printf("\n");
        for (int i = 0; i < total; i++) {
            printf("%lf ", Truck.startTime[i]);
        }
        printf("\n");
        // update completionTime
        *CompletionTime = curTime;
        // update topRoute
        Truck.top_route = total-1;

        // release var
        for (int i = 0; i < total; i++) {
            free(travelTime[i]);
        }
        free(travelTime);
        free(final_tour);
    }

}




void updatePreState(Points* listofPoints, double time) {
    // update drone first then truck to avoid errors (like the drone is not even done supplying on the truck a order, but that order already take off from the truck)
    // update the drone
    {
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            //printf("DRONE %d:\n", k);
            // loop in the sorties to see how many orders has been resupply or currently onboard


            // if drone available time < time, then the drone is not in-transit anymore
            // else not check in the last performed sortie to know whether it is in-transit or not
            if (Drone[k].availableTime < time) {
                if (Drone[k].top_onboard != -1) {
                    // update Truck.onboard and Truck.loaded first
                    for (int i = 0; i <= Drone[k].top_onboard; i++) {
                        Truck.onboard[Drone[k].onboard[i]] = 0;
                        Truck.loaded[Drone[k].onboard[i]] = 1;
                    }
                    Drone[k].top_onboard = -1;

                }
            }
            else if (Drone[k].lastPerformedSortie.setofOrders != NULL) {
                int location = Drone[k].lastPerformedSortie.meetingLocation;
                if (Drone[k].lastPerformedSortie.launchTime + droneTravelTime[location] + RESUPPLY_TIME <= time) {
                    // not in-transit anymore
                    for (int i = 0; i <= Drone[k].top_onboard; i++) {
                        Truck.onboard[Drone[k].onboard[i]] = 0;
                        Truck.loaded[Drone[k].onboard[i]] = 1;
                    }
                    Drone[k].top_onboard = -1;
                }
            }

            int idx = -1;
            for (int t = 0; t <= Drone[k].topSortie; t++) {
                SORTIE curSortie = Drone[k].listofSortie[t];
                int location = curSortie.meetingLocation;

                if (curSortie.launchTime > time) break; // this sortie has not been launched
                Drone[k].availableTime = curSortie.launchTime + 2*droneTravelTime[location]+RESUPPLY_TIME;
                idx = t; // idx is the last sortie that has launchTime <= time


                // store this sortie to be last performed sortie (can be used in case the drone is in-transit)
                Drone[k].lastPerformedSortie.meetingLocation = curSortie.meetingLocation;
                Drone[k].lastPerformedSortie.launchTime = curSortie.launchTime;
                Drone[k].lastPerformedSortie.topOrder = curSortie.topOrder;
                Drone[k].lastPerformedSortie.setofOrders = (int*)calloc(DRONE_CAPACITY, sizeof(int));
                for (int i = 0; i <= curSortie.topOrder; i++) {
                    Drone[k].lastPerformedSortie.setofOrders[i] = curSortie.setofOrders[i];
                }

                if (curSortie.launchTime + droneTravelTime[location] + RESUPPLY_TIME < time) {
                    // this means that this sortie has already been resupply the order, but maybe the drone not comeback to the depot
                    for (int i = 0; i <= curSortie.topOrder; i++) {
                        // put all the order that already been resupplied into the truck
                        Truck.loaded[curSortie.setofOrders[i]] = 1;
                        Truck.available[curSortie.setofOrders[i]] = 0;
                    }


                }
                else if (curSortie.launchTime + droneTravelTime[location] >= time) {
                    // the drone could reach to location (but has not done resupplying)
                    for (int i = 0; i <= curSortie.topOrder; i++) {
                        Truck.onboard[curSortie.setofOrders[i]] = 1;
                        Truck.available[curSortie.setofOrders[i]] = 0;
                        Drone[k].onboard[++(Drone[k].top_onboard)] = curSortie.setofOrders[i];
                    }
                }
            }
            // now update the position of the drone based on the last performed sortie
            {
                SORTIE lastSortie = Drone[k].lastPerformedSortie;

                if (Drone[k].availableTime <= time) {
                    Drone[k].curpos.x = listofPoints[0].x;
                    Drone[k].curpos.y = listofPoints[0].y;
                }
                else if (lastSortie.launchTime + droneTravelTime[lastSortie.meetingLocation] > time) {
                    // drone did not come to the meeting location
                    double next_x = listofPoints[lastSortie.meetingLocation].x;
                    double next_y = listofPoints[lastSortie.meetingLocation].y;
                    double ratio = (time - lastSortie.launchTime)/ droneTravelTime[lastSortie.meetingLocation];
                    Drone[k].curpos.x = Drone[k].curpos.x + (next_x - Drone[k].curpos.x)*ratio;
                    Drone[k].curpos.y = Drone[k].curpos.y + (next_y - Drone[k].curpos.y)*ratio;
                }
                else if (lastSortie.launchTime + droneTravelTime[lastSortie.meetingLocation] + RESUPPLY_TIME > time) {
                    // drone at the meeting location but did not yet resupply to the truck
                    Drone[k].curpos.x = listofPoints[lastSortie.meetingLocation].x;
                    Drone[k].curpos.y = listofPoints[lastSortie.meetingLocation].y;
                }
                else if (lastSortie.launchTime + 2.0*droneTravelTime[lastSortie.meetingLocation] + RESUPPLY_TIME > time) {
                    // drone is currently flying back to the depot
                    double next_x = listofPoints[lastSortie.meetingLocation].x;
                    double next_y = listofPoints[lastSortie.meetingLocation].y;
                    double ratio = (time - lastSortie.launchTime - droneTravelTime[lastSortie.meetingLocation] - RESUPPLY_TIME)
                                        /droneTravelTime[lastSortie.meetingLocation];
                    Drone[k].curpos.x = next_x + (listofPoints[0].x - next_x)*ratio;
                    Drone[k].curpos.y = next_y + (listofPoints[0].y - next_y)*ratio;
                }
            }


            // now delete all the sortie that already done, (maybe include already launched but not done)
            for (int t = idx+1; t <= Drone[k].topSortie; t++) {
                Drone[k].listofSortie[t-(idx+1)] = Drone[k].listofSortie[t];
            }
            Drone[k].topSortie -= (idx+1);
        }
    }

    // update the truck
    {
        int id = -1;
        for (int i = 0; i <= Truck.top_route; i++) {
            if (Truck.startTime[i] > time) break;
            id = i;
        }
        // update the loaded
        for (int j = 0; j <= id; j++) {
            Truck.loaded[Truck.route[j]] = 0;
        }
        // calculate Truck.curpos based on time and Truck.startTime[id]
        {
            if (id != -1) {
                Truck.curpos.x = listofPoints[Truck.route[id]].x;
                Truck.curpos.y = listofPoints[Truck.route[id]].y;
                double diff = time - Truck.startTime[id];
                double dist = id + 1 > Truck.top_route ? M*1000000.0 : truckTravelTime[Truck.route[id]][Truck.route[id+1]]; // assume truck has not done a cycle
                if (diff > 1e-5) {
                    double next_x = listofPoints[Truck.route[id+1]].x;
                    double next_y = listofPoints[Truck.route[id+1]].y;
                    double ratio = diff/dist;
                    Truck.curpos.x = (Truck.curpos.x) + (next_x - Truck.curpos.x)*ratio;
                    Truck.curpos.y = (Truck.curpos.y) + (next_y - Truck.curpos.y)*ratio;
                }
            }

        }
        // update the onboard (already done by updating drone)
        // update the available (already done by updating drone)

        // modify the route (slide the route from id+1 back to 0)
        for (int j = id+1; j <= Truck.top_route; j++) {
            Truck.route[j-(id+1)] = Truck.route[j];
            Truck.startTime[j-(id+1)] = Truck.startTime[j];
        }
        Truck.top_route -= (id+1);
    }
}


int findLocation(int* curRoute, int topRoute, Points* listofPoints, double curTime) {
    int l = -1;
    for (int k = 0; k < NUMBER_OF_DRONES; k++) {
        if (Drone[k].top_onboard != -1) {
            //printf("DRONE %d IS IN-TRANSIT\n", k);
            // this drone is currently in-transit

            // extract the last performed sortie to quickly search up for the meeting location
            int lastmeetingLocation = Drone[k].lastPerformedSortie.meetingLocation;

            //printf("   LAST LAUNCHED TIME: %lf\n", lastLaunchTime);
            //printf("   DEPOT COORDINATES: %lf %lf\n", listofPoints[0].x, listofPoints[0].y);
            //printf("   DRONE CURRENT COORDINATES: %lf %lf\n", Drone[k].curpos.x, Drone[k].curpos.y);
            //printf("   TRACE BACK LOCATION COORDINATES: %lf %lf\n", lastMeetingLocation_x, lastMeetingLocation_y);

            // starting from l to avoid calculating over and over again
            for (int i = l+1; i <= topRoute; i++) {
                //printf("   CURRENT LOCATION COORDINATES: %lf %lf\n", listofPoints[curRoute[i]].x, listofPoints[curRoute[i]].y);
                if (lastmeetingLocation == curRoute[i]) {

                    l = i;
                    break;
                }
            }
        }
    }
    // if no drones is currently in-transit, just return -1
    return l;
}

void cheapest_insertion(int* curRoute, int* topRoute, int l, Points curOrder) {
    double minDiff = 100000000.0;
    int idx = -1;
    for (int i = l+1; i <= *topRoute; i++) {
        double newDiff = truckTravelTime[curRoute[i]][curOrder.id]
                        +truckTravelTime[curRoute[i-1]][curOrder.id]
                        -truckTravelTime[curRoute[i]][curRoute[i-1]];
        if (minDiff > newDiff) {
            minDiff = newDiff;
            idx = i;
        }
    }
    // add the curOrder to the route
    for (int i = *topRoute; i >= idx; i--) {
        curRoute[i+1] = curRoute[i];
    }
    curRoute[idx] = curOrder.id;
    (*topRoute)++;
}

double calc_lowerbound(int* curRoute, int topRoute, int startTime, int numberofPoints) {
    // counting the available orders at the depot
    int cnt = 0;
    for(int i = 1; i <= numberofPoints; i++) {
        if (Truck.available[i]) cnt++;
    }
    cnt++; // assume we insert the order
    double ans = startTime;
    for (int i = 0; i < topRoute; i++) {
        ans += truckTravelTime[curRoute[i]][curRoute[i+1]];
    }

    if (cnt % DRONE_CAPACITY == 0) ans += RESUPPLY_TIME*(cnt/DRONE_CAPACITY);
    else ans += RESUPPLY_TIME*(cnt/DRONE_CAPACITY+1);
    return ans;
}

bool accepted_mechanism(double newTime, double oldTime, int curCnt, int numberofPoints, int firstAvailableCnt, bool isGreedy) {
    double ratio = isGreedy ? 1 : ((curCnt + firstAvailableCnt)*1.0)/(1.0*numberofPoints);

    double threshold = ratio*(H-oldTime);
    printf("newTime: %lf, oldTime: %lf, threshold: %lf, ratio: %lf\n", newTime, oldTime, threshold, ratio);
    return newTime - oldTime <= threshold;
}

void DRP(int* curRoute, int topRoute,
         int* availableOrders, double* availableTime,
         Points* listofPoints,
         Points curOrder, double curTime, double* CompletionTime,
         int curCnt, int firstAvailableCnt, int numberofPoints, bool isGreedy) {
    // assume all nodes can be visited based on flight-time endurance of the drone
    // -> thus I(j) is the same for all j, I(j) = D for all j in 1..maxSortie
    // -> thus J(i) is the same for all i in D

    // here we assume all scheduled drone resupply appear before node "l" is fixed
    // -> available orders for DRP problem is changed accordingly based on the orders resupplied in those tours
    // -> available time of the drone would also changed accordingly to the same thing mentioned above



    // finding the first position where all drones are available (or the last in-transit drone meeting location)
    //
    int l = findLocation(curRoute, topRoute, listofPoints, curTime);
    printf("DONE FINDING LOCATION L: %d IN DRP AT EPOCH %d\n", l, curCnt);
    bool hasDrone_intransit = false;
    if (l == -1) l = 0;
    else hasDrone_intransit = true;
    if (hasDrone_intransit) printf("THERE IS AN IN-TRANSIT DRONE\n");
    else printf("THERE IS NO IN-TRANSIT DRONES\n");
    // find the maximum location where available orders at the depot still exists
    //
    int last = -1;
    for (int i = topRoute; i >= l; i--) {
        if (availableOrders[curRoute[i]]) {
            last = i;
            break;
        }
    }
    printf("DONE FINDING LAST POSITION: %d IN DRP AT EPOCH %d\n", last, curCnt);
    // based on the paper I(j) = D = {"l", ..., last} for all j in (1,..,maxSortie)
    //                    I = {"l", ... last}
    //                    J(i) = {1, ..., maxSortie) for all i in D = {"l", .. last}
    // but we will call all the variables with i varies from 0 to topRoute, j varies from 1 to maxSortie, k from 1 to NUMBER_OF_DRONES

    // before going into the model, we calculate T~(l) - time when truck is ready to depart from node l
    // since fixing all of the schedule drone tours, truck tour before node "l", so T~(l) is simply the time the truck would start at position "l"
    double readyTimeat_l = Truck.startTime[l];
    int availableCnt = 0;
    for (int i = 0; i <= topRoute; i++) {
        if (availableOrders[curRoute[i]]) availableCnt++;
    }
    int maxSortie = (availableCnt % DRONE_CAPACITY) ? availableCnt/DRONE_CAPACITY + 1 : availableCnt/DRONE_CAPACITY;
    printf("DONE FINDING MAXSORTIE: %d FOR %d AVAILABLE ORDERS AT EPOCH %d\n", maxSortie, availableCnt, curCnt);

    // here since if node "l" is a meeting location of a in-transit drone, then future resupplies cannot appear at node "l"
    // to keep it as simple as possible, assume a new variables first to handle those two different cases;
    int first = hasDrone_intransit ? l+1 : l;

    // preparing sets
    int is_in_I_[maxSortie][topRoute+1] = {};
    int is_in_D[topRoute+1] = {};
    int is_in_I[topRoute+1] = {};
    int is_in_J[topRoute+1][maxSortie] = {};


    int top = 0; int count = 0;
    for (int i = first; i <= last; i++) {
        if (availableOrders[curRoute[i]]) {
            count++;
        }
        // if the count of available orders is larger than 0, DRONE_CAPACITY, ...,
        // then this current location is the last possible meeting for the first sortie, second sortie, ...
        for (int j = top; j < maxSortie; j++) {
            is_in_I_[j][i] = 1;
        }
        if (count % DRONE_CAPACITY == 1) top++;
    }
    for (int j = 0; j < maxSortie; j++) {
        for (int i = first; i <= last; i++) {
            if (is_in_I_[j][i]) is_in_J[i][j] = 1;
        }
    }
    int newlast;
    for (int i = first; i <= last; i++) {
        bool isIn = false;
        for (int j = 0; j < maxSortie; j++) {
            if (is_in_I_[j][i]) {
                isIn = true; break;
            }
        }
        if (isIn) {
            is_in_D[i] = is_in_I[i] = 1;
            newlast = i;
        }
    }
    last = newlast; // since last could not be the place where the last sortie can take places
    printf("DONE PREPARING SETS USED IN DRP\n");

    // now create an MILP model
    SCIP* scip = NULL;
    SCIP_SOL* sol = NULL;

    SCIP_CALL( SCIPcreate(&scip) );
    SCIP_CALL( SCIPincludeDefaultPlugins(scip) );
    SCIP_CALL( SCIPcreateProbBasic(scip, "DRP") );

    SCIP_CALL( SCIPsetIntParam(scip, "misc/usesymmetry", 0) );


    SCIP_VAR* T[topRoute+1];
    SCIP_VAR* y[maxSortie][topRoute+1];
    SCIP_VAR* u[NUMBER_OF_DRONES][topRoute+1];

    //printf("DONE CREATING VARIABLES\n");
    // create variables
    char name[64];
    /* T[i] */
    for (int i = first; i <= last; i++) {

        snprintf(name, sizeof(name), "T_%d", i);
        //printf("%s\n", name);
        if (i == last) SCIP_CALL(SCIPcreateVarBasic(scip, &T[i], name, 0.0, SCIPinfinity(scip), 1.0, SCIP_VARTYPE_CONTINUOUS));
        else SCIP_CALL(SCIPcreateVarBasic(scip, &T[i], name, 0.0, SCIPinfinity(scip), 0.0, SCIP_VARTYPE_CONTINUOUS));
        SCIP_CALL(SCIPaddVar(scip, T[i]));
    }

    /* y[j][i] */
    for (int j = 0; j < maxSortie; j++) {
        for (int i = first; i <= last; i++) {
            snprintf(name, sizeof(name), "y_%d_%d", j, i);
            //printf("%s\n", name);
            SCIP_CALL(SCIPcreateVarBasic(scip, &y[j][i], name, 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY));
            SCIP_CALL(SCIPaddVar(scip, y[j][i]));
        }
    }

    /* u[k][i] */
    for (int k = 0; k < NUMBER_OF_DRONES; k++) {
        for (int i = first; i <= last; i++) {
            snprintf(name, sizeof(name), "u_%d_%d", k, i);
           // printf("%s\n", name);
            SCIP_CALL(SCIPcreateVarBasic(scip, &u[k][i], name, 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY));
            SCIP_CALL(SCIPaddVar(scip, u[k][i]));
        }
    }

    SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

    //printf("DONE INITIALIZING VARIABLES\n");

    int MAX_NUMBER_OF_VARIABLES = (topRoute+1)*maxSortie*NUMBER_OF_DRONES;
    SCIP_VAR* vars[MAX_NUMBER_OF_VARIABLES];
    double coefs[MAX_NUMBER_OF_VARIABLES];

    // SCIPallocBufferArray(scip, &vars, MAX_NUMBER_OF_VARIABLES);
    // SCIPallocBufferArray(scip, &coefs, MAX_NUMBER_OF_VARIABLES);

    SCIP_CONS* cons;

    int cnt;



    // constraint 53: sum_(i in I(j)){y[j][i]} = 1 for j in J
    {
        for (int j = 0; j < maxSortie; j++) {
            cnt = 0;
            for (int i = 0; i <= topRoute; i++) {
                if (is_in_I_[j][i]) {
                    vars[cnt] = y[j][i]; coefs[cnt] = 1.0; cnt++;
                }
            }
            snprintf(name, sizeof(name), "c53_%d", j);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 1.0, 1.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIP_CALL(SCIPreleaseCons(scip, &cons));
        }
    }
    //printf("DONE ADDING CONSTRAINT 53\n");

    // constraint 54: y[j'][i'] <= 1 - y[j][i] for j, j' in J, i in I(j), i' in I(j'), i >= i', j' > j
    {
        for (int j = 0; j < maxSortie; j++) {
            for (int j1 = j+1; j1 < maxSortie; j1++) {
                for (int i = 0; i <= topRoute; i++) {
                    if (is_in_I_[j][i]) {
                        for (int i1 = 0; i1 <= i; i1++) {
                            if (is_in_I_[j1][i1]) {
                                cnt = 0;
                                vars[cnt] = y[j1][i1]; coefs[cnt] = 1.0; cnt++;
                                vars[cnt] = y[j][i]; coefs[cnt] = 1.0; cnt++;
                                snprintf(name, sizeof(name), "c54_%d_%d__%d_%d", j, i, j1, i1);
                                SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, -SCIPinfinity(scip), 1.0));
                                SCIP_CALL(SCIPaddCons(scip, cons));
                                SCIP_CALL(SCIPreleaseCons(scip, &cons));
                            }
                        }
                    }
                }
            }
        }
    }
    //printf("DONE ADDING CONSTRAINT 54\n");

    // constraint 55: sum_(j in J(i))y[j][i] = sum_(k in K)u[k][i] for i in D
    {
        for (int i = 0; i <= topRoute; i++) {
            if (is_in_D[i]) {

                cnt = 0;
                for (int j = 0; j < maxSortie; j++) {
                    if (is_in_J[i][j]) {
                        vars[cnt] = y[j][i]; coefs[cnt] = 1.0; cnt++;
                    }
                }

                for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                    vars[cnt] = u[k][i]; coefs[cnt] = -1.0; cnt++;
                }

                snprintf(name, sizeof(name), "c55_%d", i);
                SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, 0.0));

                SCIP_CALL(SCIPaddCons(scip, cons));

                SCIP_CALL(SCIPreleaseCons(scip, &cons));

            }
        }

    }

    //printf("DONE ADDING CONSTRAINT 55\n");

    // constraint 56: T["l"] >= readyTime_at_l + RESUPPLY_TIME*(sum_(k in K)u[k]["l"]) (apply for no in-transit drones)
    //                T[first] >= readyTime_at_l + RESUPPLY_TIME*(sum_(k in K)u[k][first] + truck_time[first][first-1]
    //             (apply for in-transit drones existing)
    {
        if (!hasDrone_intransit) {
            cnt = 0;
            vars[cnt] = T[l]; coefs[cnt] = 1.0; cnt++;
            for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                vars[cnt] = u[k][l]; coefs[cnt] = -1.0*RESUPPLY_TIME; cnt++;
            }
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, "c56", cnt, vars, coefs, readyTimeat_l, SCIPinfinity(scip)));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIP_CALL(SCIPreleaseCons(scip, &cons));
        }
        else {
            cnt = 0;
            vars[cnt] = T[first]; coefs[cnt] = 1.0; cnt++;
            for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                vars[cnt] = u[k][first]; coefs[cnt] = -1.0*RESUPPLY_TIME; cnt++;
            }
            double lhs = readyTimeat_l + truckTravelTime[curRoute[first]][curRoute[first-1]];
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, "c56", cnt, vars, coefs, lhs, SCIPinfinity(scip)));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIP_CALL(SCIPreleaseCons(scip, &cons));
        }
    }

    //printf("DONE ADDING CONSTRAINT 56\n");

    // constraint 57: T[i] >= T[i-1] + truck_time[i-1][i] for i in I\D (technically never happened since I is a subset of D)
    {
        for (int i = 0; i <= topRoute; i++) {
            if (is_in_I[i] && !is_in_D[i] && i >= 1) {
                cnt = 0;
                vars[cnt] = T[i]; coefs[cnt] = 1.0; cnt++;
                vars[cnt] = T[i-1]; coefs[cnt] = -1.0; cnt++;
                double lhs = truckTravelTime[curRoute[i]][curRoute[i-1]];
                snprintf(name, sizeof(name), "c57_%d", i);
                SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, lhs, SCIPinfinity(scip)));
                SCIP_CALL(SCIPaddCons(scip, cons));
                SCIP_CALL(SCIPreleaseCons(scip, &cons));
            }
        }
    }

    //printf("DONE ADDING CONSTRAINT 57\n");

    // constraint 58: T[i] >= T[i-1] + truck_time[i-1][i] + RESUPPLY_TIME*(sum_(k in K)u[k][i]) for i in D \ {"l"} , i in I
    {
        for (int i = 0; i <= topRoute; i++) {
            if (is_in_D[i] && is_in_I[i] && i != first) {
                cnt = 0;
                vars[cnt] = T[i]; coefs[cnt] = 1.0; cnt++;
                vars[cnt] = T[i-1]; coefs[cnt] = -1.0; cnt++;
                for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                    vars[cnt] = u[k][i]; coefs[cnt] = -1.0*RESUPPLY_TIME; cnt++;
                }
                double lhs = truckTravelTime[curRoute[i]][curRoute[i-1]];
                snprintf(name, sizeof(name), "c58_%d", i);
                SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, lhs, SCIPinfinity(scip)));
                SCIP_CALL(SCIPaddCons(scip, cons));
                SCIP_CALL(SCIPreleaseCons(scip, &cons));
            }
        }
    }
    //printf("DONE ADDING CONSTRAINT 58\n");

    // constraint 59: T[i] >= (max(a[k], curTime) + drone_time[i] + RESUPPLY_TIME)*u[k][i] for i in D, k in K
    // modified this a little bit to prevent predicting new orders
    {
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int i = 0; i <= topRoute; i++) {
                if (is_in_D[i] && is_in_I[i]) {
                    cnt = 0;
                    vars[cnt] = T[i]; coefs[cnt] = 1.0; cnt++;
                    double holder = availableTime[k] > curTime ? availableTime[k] : curTime;
                    vars[cnt] = u[k][i]; coefs[cnt] = -1.0*(holder + droneTravelTime[curRoute[i]] + RESUPPLY_TIME); cnt++;
                    snprintf(name, sizeof(name), "c59_%d_%d", k, i);
                    SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, SCIPinfinity(scip)));
                    SCIP_CALL(SCIPaddCons(scip, cons));
                    SCIP_CALL(SCIPreleaseCons(scip, &cons));
                }
            }
        }
    }

    //printf("DONE ADDING CONSTRAINT 59\n");

    // constraint 60: T[i] >= T[j] + (drone_time[i] + RESUPPLY_TIME + drone_time[j])*(u[k][i] + u[k][j] - 1)
    //                      for k in K, i in D\{"l"}, j in D, j < i
    {
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int i = 0; i <= topRoute; i++) {
                if (is_in_D[i] && is_in_I[i] && i != first) {
                    for (int j = 0; j < i; j++) {
                        if (is_in_D[j] && is_in_I[j]) {
                            cnt = 0;
                            vars[cnt] = T[i]; coefs[cnt] = 1.0; cnt++;
                            vars[cnt] = T[j]; coefs[cnt] = -1.0; cnt++;
                            vars[cnt] = u[k][i]; coefs[cnt] = -1.0*(droneTravelTime[curRoute[i]] + RESUPPLY_TIME + droneTravelTime[curRoute[j]]); cnt++;
                            vars[cnt] = u[k][j]; coefs[cnt] = -1.0*(droneTravelTime[curRoute[i]] + RESUPPLY_TIME + droneTravelTime[curRoute[j]]); cnt++;
                            snprintf(name, sizeof(name), "c60_%d_%d_%d", k, i, j);
                            double lhs = -1.0*(droneTravelTime[curRoute[i]] + RESUPPLY_TIME + droneTravelTime[curRoute[j]]);
                            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, lhs, SCIPinfinity(scip)));
                            SCIP_CALL(SCIPaddCons(scip, cons));
                            SCIP_CALL(SCIPreleaseCons(scip, &cons));
                        }
                    }
                }
            }
        }

    }

    //printf("DONE ADDING CONSTRAINT 60\n");

    // constraint 64: a valid lower_bound
    // T[last] >= readyTime_at_l + sum_(i in I\{"l"})truck_time[i][i-1] + RESUPPLYTIME*maxSortie
    {
        cnt = 0;
        vars[cnt] = T[last]; coefs[cnt] = 1.0; cnt++;
        double lhs = readyTimeat_l + RESUPPLY_TIME*maxSortie;
        for (int i = l+1; i <= last; i++) {
            lhs += truckTravelTime[curRoute[i]][curRoute[i-1]];
        }
        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, lhs, SCIPinfinity(scip)));
        SCIP_CALL(SCIPaddCons(scip, cons));
        SCIP_CALL(SCIPreleaseCons(scip, &cons));
    }

    //printf("DONE ADDING CONSTRAINTS IN DRP\n");
    SCIP_CALL( SCIPsolve(scip) );
    sol = SCIPgetBestSol(scip);
    double newCompletionTime = SCIPgetSolOrigObj(scip, sol);
    for (int i = last+1; i <= topRoute; i++) {
        newCompletionTime += truckTravelTime[curRoute[i]][curRoute[i-1]];
    }

    // now test if this newCompletionTime is feasible by using an accepted mechanism
    bool isAccepted = accepted_mechanism(newCompletionTime, *CompletionTime, curCnt, numberofPoints, firstAvailableCnt, isGreedy);

    // if (sol) printf("SOLUTION FOUND\n");

    if (isAccepted) {
        printf("\n\n ORDER ACCEPTED \n\n");
        // if the order is accepted after solving DRP, update the state of drone and truck based on the decision variables of MILP

        // first updated the truck departure time at those nodes (after "l")
        for (int i = first; i <= last; i++) {
            Truck.startTime[i] = SCIPgetSolVal(scip, sol, T[i]);
        }
        for (int i = last+1; i <= topRoute; i++) {
            Truck.startTime[i] = Truck.startTime[i-1] + truckTravelTime[curRoute[i]][curRoute[i-1]];
        }
        // change the route to a new route (the route after doing insertion)
        for (int i = 0; i <= Truck.top_route; i++) {
            Truck.route[i] = curRoute[i];
        }
        Truck.route[++Truck.top_route] = curRoute[topRoute];
        // update list of available orders at the depot by including new order
        Truck.available[curOrder.id] = 1;
        Truck.visited[curOrder.id] = 1;
        printf("DONE UPDATING TRUCK\n");
        // now updating the state of drone using decision variables u[k][i], y[j][i], maybe using T[i] if needed
        // loop through all nodes after "l" to check for new sortie

        // make an extra array storing whether a location appears before node "l" in the curRoute
        int is_before_l[numberofPoints+2] = {};
        for (int i = 0; i < first; i++) {
            is_before_l[curRoute[i]] = 1;
        }
        // make an extra array storing the pointers to the furthest sortie in each drone
        int top[NUMBER_OF_DRONES] = {};
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int j = 0; j <= Drone[k].topSortie; j++) {
                int location = Drone[k].listofSortie[j].meetingLocation;
                if (!is_before_l[location]) {
                    top[k] = j;
                    break;
                }
            }
        }
        for (int i = first; i <= last; i++) {
            for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                // which means there is a drone sortie meeting at this location
                if (SCIPgetSolVal(scip, sol, u[k][i]) > 0.5) {
                    // add this sortie to the list_of_sortie

                    //printf("HI\n");

                    // using top[k] to add new sortie,
                    // since top[k] is currently denoted the first old sortie has the meeting location appeared after "l"

                    SORTIE newSortie;
                    newSortie.meetingLocation = curRoute[i];
                    // the launchTime is defined as T[i] - drone_time[i] - RESUPPLY_TIME
                    // so that the drone meet at the truck perfectly
                    // there could also be other launchTime, but just need to be larger than availableTime from previous sortie
                    newSortie.launchTime = Truck.startTime[i] - droneTravelTime[curRoute[i]] - RESUPPLY_TIME;

                    // to be simple, setofOrders simply including all availableOrders at the depot counting from the left of the route
                    // after doing all previous sortie, but not exceeding the DRONE_CAPACITY
                    newSortie.setofOrders = (int*)calloc(DRONE_CAPACITY, sizeof(int));
                    newSortie.topOrder = -1;
                    for (int i = 0; i <= topRoute; i++) {
                        if (newSortie.topOrder == DRONE_CAPACITY-1) break;
                        if (availableOrders[curRoute[i]]) {
                            // mark it as not available for future adding
                            newSortie.setofOrders[++newSortie.topOrder] = curRoute[i];
                            availableOrders[curRoute[i]] = 0;
                        }
                    }
                    printf("THIS SORTIE IS MEET AT %d, LAUNCHED AT %lf\n", newSortie.meetingLocation, newSortie.launchTime);
                    Drone[k].listofSortie[top[k]++] = newSortie;
                }
            }
        }

        // after insert all sortie, set back the current topSortie based on top[k]
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            Drone[k].topSortie = top[k]-1;
        }

        // finally updated the completionTime
        *CompletionTime = newCompletionTime;
    }
    else printf("\n\n ORDER DENIED \n\n");
    // free the variables
    {

        /* T[i] */
        for (int i = first; i <= last; i++) {
            SCIP_CALL(SCIPreleaseVar(scip, &T[i]));
        }
        /* u[k][i] */
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int i = first; i <= last; i++) {
                SCIP_CALL(SCIPreleaseVar(scip, &u[k][i]));
            }
        }
        /* y[j][i] */
        for (int j = 0; j < maxSortie; j++) {
            for (int i = first; i <= last; i++) {
                SCIP_CALL(SCIPreleaseVar(scip, &y[j][i]));
            }
        }
    }
    printf("DONE RELEASING VARIABLES\n");
    SCIP_CALL(SCIPfree(&scip));
}



void doingAction(Points curOrder, int* curRoute, int* topRoute,
                 Points* listofPoints, int numberofPoints, bool isGreedy,
                 double curTime, double* CompletionTime, int curCnt, int firstAvailableCnt) {

    // here we modified the algorithm a bit: we only apply insertion for node from "l" to the end
    // "l" is define as a location where the last in-transit drone will meet the truck
    int l = findLocation(curRoute, *topRoute, listofPoints, curTime);
    printf("DONE FINDING LOCATION L: %d AT EPOCH %d\n", l, curCnt);
    bool hasDrone_intransit = false;
    if (l == -1) l = 0;
    else hasDrone_intransit = true;

    // doing cheapest insertion technique
    cheapest_insertion(curRoute, topRoute, l, curOrder);
   // printf("DONE INSERTING AT EPOCH %d\n", curCnt);
    printf("  NEW ROUTE IS: ");
    for (int i = 0; i <= *topRoute; i++) {
        printf("%d ", curRoute[i]);
    }
    printf("\n");
    double startTime = Truck.startTime[0];
    // calculate the lower_bound of the tour
    double lb = calc_lowerbound(curRoute, *topRoute, startTime, numberofPoints);
    printf("DONE CALCULATING LOWER BOUND: %lf AT EPOCH %d\n", lb, curCnt);
    if (lb > H) {
        printf("\n\n   ORDER DENIED \n\n");
        return;
    }
    // in case the lb is satisfied -> invoke the DRP problem

    // but first, we create availableOrders arr for list of avaiableOrders after considering all scheduled tours at location before "l"
    // and we create availableTime arr for the drone available time after considering the same thing

    int* availableOrders = (int*)calloc(numberofPoints+2, sizeof(int));
    double* availableTime = (double*)calloc(NUMBER_OF_DRONES, sizeof(double));


    // initialize availableOrders by copying the current Truck.available arr (it is exactly that if "l" is not existed)
    for (int i = 0; i < numberofPoints; i++) {
        availableOrders[i] = Truck.available[i];
    }
    availableOrders[curOrder.id] = 1;
    // initialize availableTime by copying the current availableTime of each drone (it is exactly that if "l" is not existed)
    for (int k = 0; k < NUMBER_OF_DRONES; k++) {
        availableTime[k] = Drone[k].availableTime;
    }
    // in case "l" exists (but "l" can still equal to 0), so add "l" != 0 to reduce unnecessary computations
    if (hasDrone_intransit && l != 0) {
        // this helps checking whether the meeting location is before "l" or not
        bool is_before_node_l[numberofPoints+2] = {};
        for (int i = 0; i < l; i++) {
            int location = curRoute[i];
            is_before_node_l[location] = 1;
        }

        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            // if the drone is in-transit, future schedule tours could also appear before node "l", so we still need to consider it
            // the resupply orders that are currently onboard are already updated in the updatePreState(), so we do not need to check for those orders
            for (int j = 0; j <= Drone[k].topSortie; j++) {
                int location = Drone[k].listofSortie[j].meetingLocation;
                // note that except when there is no in-transit drones,
                // the location would never be equal to "l", since there is a in-transit drone that is gonna meet the truck at "l"
                if (is_before_node_l[location]) {
                    // then those orders in this sortie would turn to be unavailable at the depot
                    for (int t = 0; t <= Drone[k].listofSortie[j].topOrder; t++) {
                        int order = Drone[k].listofSortie[j].setofOrders[t];
                        availableOrders[order] = 0;
                        // update availableTime assumed that this sortie is fixed
                        availableTime[k] = Drone[k].listofSortie[j].launchTime + 2*droneTravelTime[location] + RESUPPLY_TIME;
                    }
                }
                // since this is the first sortie whose meeting location is after "l"
                else break;
            }
        }
    }
    //printf("DONE PREPARING STUFF BEFORE SOLVING DRP AT EPOCH %d\n", curCnt);
    printf("Available Orders in DRP: ");
    for (int i = 1; i <= numberofPoints; i++) {
        if (availableOrders[i]) printf("%d ", i);
    }
    printf("\n");
    printf("Available Time of Drone: \n");
    for (int k = 0; k < NUMBER_OF_DRONES; k++) {
        printf("   Drone %d: %lf\n", k, availableTime[k]);
    }
    DRP(curRoute, *topRoute, availableOrders, availableTime, listofPoints, curOrder, curTime, CompletionTime,
        curCnt, firstAvailableCnt, numberofPoints, isGreedy);

    // release Var
    {
        free(availableOrders);
        free(availableTime);
    }

   // printf("DONE DRP AND UPDATING AT EPOCH %d\n", curCnt);
}

void printState(int numberofPoints, double time) {
    // print Truck info
    printf("EPOCH: \n");
    printf("  TRUCK ROUTE: ");
    for (int i = 0; i <= Truck.top_route; i++) {
        printf("%d ", Truck.route[i]);
    }
    printf("\n");
    printf("  START TIME: ");
    for (int i = 0; i <= Truck.top_route; i++) {
        printf("%lf ", Truck.startTime[i]);
    }
    printf("\n");
    printf("  LOADED ORDERS: ");
    for (int i = 1; i <= numberofPoints; i++) {
        if (Truck.loaded[i])
            printf("%d ", i);
    }
    printf("\n");

    printf("  AVAILABLE ORDERS AT DEPOT: ");
    for (int i = 1; i <= numberofPoints; i++) {
        if (Truck.available[i])
            printf("%d ", i);
    }
    printf("\n");
    printf("  ONBOARD ORDERS: ");
    for (int i = 1; i <= numberofPoints; i++) {
        if (Truck.onboard[i]) printf("%d ", i);
    }
    printf("\n\n");


    // print Drone info
    for (int k = 0; k < NUMBER_OF_DRONES; k++) {
        printf("  Drone %d:\n", k);
        if (Drone[k].availableTime < time) printf("    Drone is now at depot \n");
        else if (Drone[k].top_onboard != -1) {
            printf("    Currently in-transit: ");
            for (int t = 0; t <= Drone[k].top_onboard; t++) {
                printf("%d ", Drone[k].onboard[t]);
            }
            printf("\n");
        }
        else printf("    Currently not in-transit but not back to depot\n");
        printf("    LIST OF SORTIE: ");
        if (Drone[k].topSortie == -1) printf("NONE\n");
        else printf("\n");
        for (int j = 0; j <= Drone[k].topSortie; j++) {
            printf("      Sortie %d: \n", j);
            printf("         Meeting location: %d \n", Drone[k].listofSortie[j].meetingLocation);
            printf("         Launch Time: %lf \n", Drone[k].listofSortie[j].launchTime);
            printf("         Orders: ");
            for (int i = 0; i <= Drone[k].listofSortie[j].topOrder; i++) {
                printf("%d ", Drone[k].listofSortie[j].setofOrders[i]);
            }
            printf("\n");
        }
        printf("    AVAILABLE AT THE DEPOT AT %lf:\n", Drone[k].availableTime);
    }
}

int cmp(const void* x, const void* y) {
    return ((Points*)x)->releaseTime - ((Points*)y)->releaseTime;
}

void read_input(const char* filename, Points* listofPoints, int* numberofPoints) {
    FILE *fp = fopen(filename, "r");
    if(!fp) {
        fprintf(stderr, "Error opening \"%s\": %s\n", filename, strerror(errno));
        return;
    }

    // Read and discard the header line
    char line[256];
    if(!fgets(line, sizeof(line), fp)) {
        fprintf(stderr, "Empty file or read error on header: %s\n", strerror(errno));
        fclose(fp);
        return;
    }

    while(*numberofPoints < MAXPOINTS && fgets(line, sizeof(line), fp)) {
        // Skip empty lines
        if(line[0] == '\n' || line[0] == '\r')
            continue;

        // Parse four fields: id, x, y, r
        int id;
        double x, y, r;
        int scanned = sscanf(line, "%d\t%lf\t%lf\t%lf", &id, &x, &y, &r);
        if(scanned != 4) {
            // Try fallback: whitespace-separated
            scanned = sscanf(line, "%d %lf %lf %lf", &id, &x, &y, &r);
        }
        if(scanned == 4) {
            listofPoints[*numberofPoints].id = id;
            listofPoints[*numberofPoints].x  = x;
            listofPoints[*numberofPoints].y  = y;
            listofPoints[*numberofPoints].releaseTime  = r;
            (*numberofPoints)++;
        } else {
            fprintf(stderr, "Warning: skipping malformed line %d: %s", (*numberofPoints)+2, line);
        }
    }

    fclose(fp);
    return;
}

// this function generates a horizon for the problem based on the formula zeta*(beta*z(TSP))
double calculate_horizon(Points* listofPoints, int numberofPoints, double* listofZeta, int option) {
    double zeta = listofZeta[option-1];
    double scalar = 1.0;
    double** dist_matrix = (double**)calloc(numberofPoints+2, sizeof(double*));
    for (int i = 0; i < numberofPoints+2; i++) {
        dist_matrix[i] = (double*)calloc(numberofPoints+2, sizeof(double));
        for (int j = 0; j < numberofPoints+2; j++) {
            dist_matrix[i][j] = truckTravelTime[i][j];
        }
    }
    int* final_tour = (int*)calloc(numberofPoints+2, sizeof(int));
    TSP_heuristic(dist_matrix, numberofPoints+2, final_tour);
    double totalTime = 0.0;
    for (int i = 0; i < numberofPoints+1; i++) {
        totalTime += dist_matrix[final_tour[i]][final_tour[i+1]];
    }
    free(final_tour);
    for (int i = 0; i < numberofPoints+2; i++) {
        free(dist_matrix[i]);
    }
    free(dist_matrix);
    return zeta*scalar*totalTime;
}

double listofHorizon[] = {632.0, 505.13, 529.05, 615.02};
double listofZeta[] = {1.0, 1.25, 1.5};
char listofInstanceName[][25] = {"R101", "C101", "C201", "RC101"};

FILE* out;

void solve_instance(char* filename, double horizon, double zeta, char* name, int instance_id, bool isGreedy, bool isTable1) {
    Points* listofPoints = (Points*)malloc(sizeof(Points)*MAXPOINTS);
    int numberofPoints = 0;

    H = horizon*zeta;
    printf("Running %s: with zeta = %lf:\n", filename, zeta);
    delay(3);

    read_input(filename, listofPoints, &numberofPoints);


    if (numberofPoints == 0) return;

    numberofPoints--; // since this includes depot, subtract it by 1
    listofPoints[numberofPoints+1] = listofPoints[0];
    // print the data
    for (int i = 0; i <= numberofPoints; i++) {
        printf("%d %lf %lf %lf\n", listofPoints[i].id, listofPoints[i].x, listofPoints[i].y, listofPoints[i].releaseTime);
    }


    // initialize the time travel of truck and drones between nodes
    initTravelTime(listofPoints, numberofPoints);


    // initialize the properties of truck and drones
    initTruck(listofPoints, numberofPoints);
    initAllDrone(listofPoints[0], numberofPoints);


    /// if you want to calculate the upper bound, remove the "//"
    // OPRD_DR(listofPoints, numberofPoints); // find upper bound of the problem
    delay(5);
    ///
    ///

    double CompletionTime;
    // make a copy of the listofPoints
    Points* copyList = (Points*)malloc(sizeof(Points)*(numberofPoints+2));
    for (int i = 0; i < numberofPoints+2; i++) {
        copyList[i] = listofPoints[i];
    }
    // sort the point according to release time, to stimulate the process
    qsort(&copyList[1], numberofPoints, sizeof(Points), cmp);
    int cnt = 0;
    for (int i = 1; i <= numberofPoints; i++) {
        if (copyList[i].releaseTime != 0) break;
        cnt++;
    }
    Points* InitialAvailablePoints = (Points*)malloc(sizeof(Points)*(cnt+2));
    // copy data
    for (int i = 0; i <= cnt; i++) {
        InitialAvailablePoints[i] = copyList[i];
    }
    InitialAvailablePoints[cnt+1] = listofPoints[0];

    for (int i = 0; i <= cnt+1; i++) {
        printf("%d ", InitialAvailablePoints[i].id);
    }
    printf("\n");

    initTruckRoute(InitialAvailablePoints, cnt, &CompletionTime); // solving TSP

    //return 0;

    int firstAvailableCnt = cnt;
    int cur_epoch = cnt+1;

    int* curRoute = (int*)malloc(sizeof(int)*numberofPoints);
    int topRoute;
    int curCnt = 0;

    // those things are used to record the decision-time
    struct timespec start, end;
    double total_decision_time = 0.0; // store the total_time
    double max_decision_time = 0.0; // store the max decision time


    while (cur_epoch < numberofPoints+1 && copyList[cur_epoch].releaseTime < H) {

        if (Truck.top_route <= 0) break; // the tour is ended

        // Record start time
        if (clock_gettime(CLOCK_MONOTONIC, &start) != 0) {
            perror("clock_gettime");
            return;
        }

        double curTime = copyList[cur_epoch].releaseTime;
        curCnt++; // the number of dynamic orders seen so far
        printf("NEW ORDER IS %d\n", copyList[cur_epoch].id);



        updatePreState(listofPoints, curTime); // update the state at curTime
        // copy the route after updating the state;
        printf("\nDONE UPDATING STATE at epoch %d\n", curCnt);
        printf("CURRENT TIME IS %lf\n", curTime);
        printState(numberofPoints, curTime);

        if (Truck.top_route <= 0) break; // the tour is ended


        topRoute = -1;
        for (int i = 0; i <= Truck.top_route; i++) {
            curRoute[++topRoute] = Truck.route[i];
        }
        // doing action
        Points curOrder = copyList[cur_epoch];

        doingAction(curOrder, curRoute, &topRoute,
                 listofPoints, numberofPoints, isGreedy,
                 curTime, &CompletionTime, curCnt, firstAvailableCnt);

        printf("\nDONE DOING ACTION AT EPOCH %d\n", curCnt);
        printf("CURRENT TIME IS %lf\n", curTime);
        printState(numberofPoints, curTime);

        printf("\nPRESS ENTER TO MOVE TO THE NEXT EPOCH: \n");




        // Record end time
        if (clock_gettime(CLOCK_MONOTONIC, &end) != 0) {
            perror("clock_gettime");
            return;
        }

        // Calculate elapsed time in nanoseconds
        long elapsed_ns = (end.tv_sec - start.tv_sec) * 1000000000L
                        + (end.tv_nsec - start.tv_nsec);

        // Print iteration and elapsed time
        printf("Iteration %d: %ld ns (%.3f ms)\n", curCnt + 1, elapsed_ns,
               elapsed_ns / 1e6);

        max_decision_time = (elapsed_ns / 1e9) > max_decision_time ? (elapsed_ns / 1e9) : max_decision_time;
        total_decision_time += (elapsed_ns / 1e9);
        cur_epoch++; // move on to the next epoch
    }

    int ans = 0; // store the objective
    for (int i = 1; i <= numberofPoints; i++) {
        if (Truck.visited[i]) ans++;
    }
    printf("THE NUMBER OF ORDERS: %d\n", ans);

    char policyType[99];
    if (isGreedy) strcpy(policyType, "Greedy");
    else strcpy(policyType, "Adaptive");

    if (isTable1) fprintf(out, "\n%s         | %d         | %.2lf            | %d             | %.0lf          | %.3lf               | %.3lf        ",
                        name, instance_id, zeta, ans, CompletionTime, total_decision_time/curCnt, max_decision_time);
    else fprintf(out, "\n%s      | %s        | %d         | %d         ", policyType, name, instance_id, ans);
    // release Var
    {
        // free truck info
        free(Truck.available);
        free(Truck.loaded);
        free(Truck.onboard);
        free(Truck.route);
        free(Truck.visited);
        free(Truck.startTime);

        //free drone info
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            free(Drone[k].onboard);
            for (int j = 0; j < Drone[k].topSortie; j++) {
                free(Drone[k].listofSortie[j].setofOrders);
            }
            free(Drone[k].listofSortie);
            if (Drone[k].lastPerformedSortie.setofOrders) free(Drone[k].lastPerformedSortie.setofOrders);
        }

        free(curRoute);
        free(listofPoints);
        free(copyList);
        free(InitialAvailablePoints);
    }


}

int main() {



    out = NULL;
    out = fopen("results.txt", "a");

    if(!out) {
        fprintf(stderr, "Error opening \"%s\": %s\n", "results.txt", strerror(errno));
        return -1;
    }


    // fprintf(out, "Distribution  | Instance | Zeta | Objective | Total tour time | Average decision time (s) | Maximum decision time (s)");

    // solve_instance("Instances/C101_100_75_1.txt", 505.13, 1.0, "C101", 1);
    for (int i = 4; i < 4; i++) {
        for (int zeta_id = 0; zeta_id < 3; zeta_id++) {
            double zeta = listofZeta[zeta_id];
            for (int id = 1; id <= 10; id++) {
                char name[90];
                strcpy(name, "Instances/");
                strcat(name, listofInstanceName[i]);
                strcat(name, "_100_75_");
                double Horizon = listofHorizon[i];
                char index[5];
                sprintf(index, "%d.txt", id);
                strcat(name, index);
                solve_instance(name, Horizon, zeta, listofInstanceName[i], id, false, true);
            }

        }
    }
    fclose(out);
    //return 0;

    // print the second table
    out = NULL;
    out = fopen("result1.txt", "a");
    if(!out) {
        fprintf(stderr, "Error opening \"%s\": %s\n", "results.txt", strerror(errno));
        return -1;
    }
    for (int i = 0; i < 4; i++) {
        double zeta = listofZeta[0];
        double Horizon = listofHorizon[i];
        for (int id = 1; id <= 10; id++) {
            char name[90];
            strcpy(name, "Instances/");
            strcat(name, listofInstanceName[i]);
            strcat(name, "_100_75_");
            char index[5];
            sprintf(index, "%d.txt", id);
            strcat(name, index);
            solve_instance(name, Horizon, zeta, listofInstanceName[i], id, false, false);
        }
    }
    for (int i = 0; i < 4; i++) {
        double zeta = listofZeta[0];
        double Horizon = listofHorizon[i];
        for (int id = 1; id <= 10; id++) {
            char name[90];
            strcpy(name, "Instances/");
            strcat(name, listofInstanceName[i]);
            strcat(name, "_100_75_");
            char index[5];
            sprintf(index, "%d.txt", id);
            strcat(name, index);
            solve_instance(name, Horizon, zeta, listofInstanceName[i], id, true, false);
        }
    }

    fclose(out);
    return 0;
}



