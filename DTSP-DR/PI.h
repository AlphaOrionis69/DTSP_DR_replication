// calculate the upper bound of the problem
void OPRD_DR(Points* listofPoints, int numberofPoints) {
    // not sorted (id is the same as index in arr)

    bool is_in_A0[numberofPoints+2]; /* A0: the set of available orders at the depot */
    bool is_in_D[numberofPoints+2]; /* D: the set of available locations to resupply (according to flight endurance) */
    for (int i = 1; i <= numberofPoints; i++) {
        if (listofPoints[i].releaseTime == 0) {
            is_in_A0[i] = true;
        }
        else is_in_A0[i] = false;

    }
    for (int i = 1; i <= numberofPoints; i++) {
        is_in_D[i] = true;
    }
    is_in_D[0] = is_in_D[numberofPoints+1] = false;

    SCIP* scip = NULL;
    SCIP_SOL* sol = NULL;

    SCIP_CALL( SCIPcreate(&scip) );
    SCIP_CALL( SCIPincludeDefaultPlugins(scip) );
    SCIP_CALL( SCIPcreateProbBasic(scip, "OPRD_DR") );

    double timelimit = 1000;
    double nodeLimit = 5000;
    // add limits time or nodes to the models
    SCIP_CALL( SCIPsetRealParam(scip, "limits/time",    timelimit) );
    SCIP_CALL( SCIPsetLongintParam(scip, "limits/nodes", nodeLimit) );



    SCIP_VAR* x[numberofPoints+2][numberofPoints+2];    /* x[i][j] = 1 if truck goes from i to j */
    SCIP_VAR* z[numberofPoints+2];       /* z[i] = 1 if order i is delivered, i in D */
    SCIP_VAR* u[NUMBER_OF_DRONES][numberofPoints+2];    /* u[k][i] = 1 if drone k visits node i, i in N */
    // u[k][0] = 1: if drone k resupply at least once
    SCIP_VAR* w[NUMBER_OF_DRONES][numberofPoints+2][numberofPoints+2]; /* w[k][i][j] if drone k travel: i -> 0 -> j; i, j in N */
    // w[k][0][j] = 1 if drone k perform the first sortie to node j
    // w[k][i][n+1] = 1 if drone k perform the last sortie to node i
    SCIP_VAR* y[NUMBER_OF_DRONES][numberofPoints+2][numberofPoints+2]; /* y[k][i][j] = load order j at node i by drone k, j <= i, j in N, i in D */
    SCIP_VAR* gamma[numberofPoints+2];    /* gamma[i] = 1 if order i is loaded at depot (i in A0) */
    SCIP_VAR* T[numberofPoints+2];      /* T[i] = departure time from node i */

    SCIP_VAR* e[numberofPoints+2]; /* e[i] = waiting time at a node i */

    char name[64];
    /* x[i][j] */
    for (int i = 0; i < numberofPoints+2; ++i) {
        for (int j = 0; j < numberofPoints+2; ++j) {
            if (i == j) continue;
            snprintf(name, sizeof(name), "x_%d_%d", i, j);
            SCIP_CALL( SCIPcreateVarBasic(scip, &x[i][j], name, 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY) );
            SCIP_CALL( SCIPaddVar(scip, x[i][j]) );
        }
    }
    /* z[i] */
    for (int i = 1; i <= numberofPoints; ++i) {
        snprintf(name, sizeof(name), "z_%d", i);
        SCIP_CALL( SCIPcreateVarBasic(scip, &z[i], name, 0.0, 1.0, 1.0, SCIP_VARTYPE_BINARY) );
        SCIP_CALL( SCIPaddVar(scip, z[i]) );
    }
    /* u[k][i] */
    for (int k = 0; k < NUMBER_OF_DRONES; ++k) {
        for (int i = 0; i <= numberofPoints; ++i) {
            if (is_in_D[i] || i == 0) {
                snprintf(name, sizeof(name), "u_%d_%d", k, i);
                SCIP_CALL( SCIPcreateVarBasic(scip, &u[k][i], name, 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY) );
                SCIP_CALL( SCIPaddVar(scip, u[k][i]) );
            }
        }
    }
    /* w[k][i][j] */
    for (int k = 0; k < NUMBER_OF_DRONES; ++k) {
        for (int i = 0; i < numberofPoints+1; ++i) {
            if (is_in_D[i] || i == 0) {
                for (int j = 1; j < numberofPoints+2; ++j) {
                    if (j != i && (is_in_D[j] || j == numberofPoints+1)) {
                        snprintf(name, sizeof(name), "w_%d_%d_%d", k, i, j);
                        SCIP_CALL( SCIPcreateVarBasic(scip, &w[k][i][j], name, 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY) );
                        SCIP_CALL( SCIPaddVar(scip, w[k][i][j]) );
                    }
                }
            }
        }
    }
    /* y[k][i][j] */
    for (int k = 0; k < NUMBER_OF_DRONES; ++k) {
        for (int i = 1; i <= numberofPoints; ++i) {
            for (int j = 1; j <= numberofPoints; ++j) {
                snprintf(name, sizeof(name), "y_%d_%d_%d", k, i, j);
                SCIP_CALL( SCIPcreateVarBasic(scip, &y[k][i][j], name, 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY) );
                SCIP_CALL( SCIPaddVar(scip, y[k][i][j]) );
            }
        }
    }
    /* gamma[i] */
    for (int i = 1; i <= numberofPoints; ++i) {
        snprintf(name, sizeof(name), "gamma_%d", i);
        SCIP_CALL( SCIPcreateVarBasic(scip, &gamma[i], name, 0.0, 1.0, 0.0, SCIP_VARTYPE_BINARY) );
        SCIP_CALL( SCIPaddVar(scip, gamma[i]) );
    }
    /* T[i] */
    for (int i = 0; i <= numberofPoints+1; ++i) {
        snprintf(name, sizeof(name), "T_%d", i);
        SCIP_CALL( SCIPcreateVarBasic(scip, &T[i], name, 0.0, SCIPinfinity(scip), 0.0, SCIP_VARTYPE_CONTINUOUS) );
        SCIP_CALL( SCIPaddVar(scip, T[i]) );
    }

    /* e[i] */
    for (int i = 1; i <= numberofPoints; i++) {
        snprintf(name, sizeof(name), "e_%d", i);
        SCIP_CALL( SCIPcreateVarBasic(scip, &e[i], name, 0.0, SCIPinfinity(scip), 0.0, SCIP_VARTYPE_CONTINUOUS) );
        SCIP_CALL( SCIPaddVar(scip, e[i]) );
    }

    // set the obj to maximize
    SCIP_CALL( SCIPsetObjsense(scip, SCIP_OBJSENSE_MAXIMIZE) );

    // adding constraints
    SCIP_CONS* cons;
    SCIP_VAR ** vars;
    double* coefs;
    long long MAX_NUMBER_OF_VARIABLES = (numberofPoints+2)*(numberofPoints+2)+2;
    SCIPallocBufferArray(scip, &vars, MAX_NUMBER_OF_VARIABLES);
    SCIPallocBufferArray(scip, &coefs, MAX_NUMBER_OF_VARIABLES);

    int cnt;

    // constraints 14 : sum_{x_0_i} = 1 for i = 1..N
    {
        cnt = 0;
        for (int i = 1; i <= numberofPoints; i++) {
            vars[cnt] = x[0][i]; coefs[cnt] = 1.0;
            cnt++;
        }
        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, "c14", cnt, vars, coefs, 1.0, 1.0));
        SCIP_CALL(SCIPaddCons(scip, cons));
        SCIPreleaseCons(scip, &cons);
    }

    // constraints 15: sum_{x_j_n+1} = 1 for j = 1..N
    {
        cnt = 0;
        for (int i = 1; i <= numberofPoints; i++) {
            vars[cnt] = x[i][numberofPoints+1]; coefs[cnt] = 1.0;
            cnt++;
        }
        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, "c15", cnt, vars, coefs, 1.0, 1.0));
        SCIP_CALL(SCIPaddCons(scip, cons));
        SCIPreleaseCons(scip, &cons);
    }


    // constraints 16: sum_j x[i][j] = z[i], (i, j) in E
    {
        for (int i = 1; i <= numberofPoints; i++) {
            cnt = 0;
            for (int j = 1; j <= numberofPoints+1; j++) {
                if (i == j) continue;
                vars[cnt] = x[i][j]; coefs[cnt] = 1.0;
                cnt++;
            }
            vars[cnt] = z[i]; coefs[cnt] = -1.0; cnt++;
            snprintf(name, sizeof(name), "c16_%d", i);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, 0.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
    }


    // constraint 17: sum_j x[j][i] = z[i], (j, i) in E
    {
        for (int i = 1; i <= numberofPoints; i++) {
            cnt = 0;
            for (int j = 0; j <= numberofPoints; j++) {
                if (j == i) continue;
                vars[cnt] = x[j][i]; coefs[cnt] = 1.0;
                cnt++;
            }
            vars[cnt] = z[i]; coefs[cnt] = -1.0; cnt++;
            snprintf(name, sizeof(name), "c17_%d", i);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, 0.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
    }


    // constraint 18: z[i] = 1 for i in A0
    {
        for (int i = 1; i <= numberofPoints; i++) {
            if (!is_in_A0[i]) continue;
            cnt = 0;
            vars[cnt] = z[i]; coefs[cnt] = 1.0; cnt++;
            snprintf(name, sizeof(name), "c18_%d", i);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 1.0, 1.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
    }

    // constraint 19: sum_k u[k][i] <= 1 for i in D
    {
        for (int i = 1; i <= numberofPoints; i++) {
            if (!is_in_D[i]) continue;
            int cnt = 0;
            for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                vars[cnt] = u[k][i]; coefs[cnt] = 1.0; cnt++;
            }
            snprintf(name, sizeof(name), "c19_%d", i);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, -SCIPinfinity(scip), 1.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
    }

    // constraint 20: u[k][i] <= u[k][0] for k in K, i in D
    {
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int i = 1; i <= numberofPoints; i++) {
                if (!is_in_D[i]) continue;
                int cnt = 0;
                vars[cnt] = u[k][i]; coefs[cnt] = 1.0; cnt++;
                vars[cnt] = u[k][0]; coefs[cnt] = -1.0; cnt++;
                snprintf(name, sizeof(name), "c20_%d_%d", k, i);
                SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, -SCIPinfinity(scip), 0.0));
                SCIP_CALL(SCIPaddCons(scip, cons));
                SCIPreleaseCons(scip, &cons);
            }
        }
    }

    // constraint 21: sum_(j in D) {w[k][0][j]} = u[k][0] for k in K
    {
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            cnt = 0;
            for (int j = 1; j <= numberofPoints; j++) {
                if (is_in_D[j]) {
                    vars[cnt] = w[k][0][j]; coefs[cnt] = 1.0; cnt++;
                }
            }
            vars[cnt] = u[k][0]; coefs[cnt] = -1.0; cnt++;
            snprintf(name, sizeof(name), "c21_%d", k);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, 0.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
    }

    // constraint 22: sum_(i in D) {w[k][i][n+1]} = u[k][0] for k in K
    {
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            cnt = 0;
            for (int i = 1; i <= numberofPoints; i++) {
                if (is_in_D[i]) {
                    vars[cnt] = w[k][i][numberofPoints+1]; coefs[cnt] = 1.0; cnt++;
                }
            }
            vars[cnt] = u[k][0]; coefs[cnt] = -1.0; cnt++;
            snprintf(name, sizeof(name), "c22_%d", k);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, 0.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
    }

    // constraint 23: sum_(j in D\{i} || j == n+1) {w[k][i][j]} = u[k][i] for k in K, i in D
    {
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int i = 1; i <= numberofPoints; i++) {
                if (is_in_D[i]) {
                    cnt = 0;
                    for (int j = 1; j <= numberofPoints+1; j++) {
                        if ((is_in_D[j] && j != i) || j == numberofPoints+1) {
                            vars[cnt] = w[k][i][j]; coefs[cnt] = 1.0; cnt++;
                        }
                    }
                    vars[cnt] = u[k][i]; coefs[cnt] = -1.0; cnt++;
                    snprintf(name, sizeof(name), "c23_%d_%d", k, i);
                    SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, 0.0));
                    SCIP_CALL(SCIPaddCons(scip, cons));
                    SCIP_CALL(SCIPreleaseCons(scip, &cons));
                }
            }
        }
    }

    // constraint 24: sum_(i in D\{j} || i == 0) {w[k][i][j]} = u[k][j] for k in K, j in D
    {
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int j = 1; j <= numberofPoints; j++) {
                if (is_in_D[j]) {
                    cnt = 0;
                    for (int i = 0; i <= numberofPoints; i++) {
                        if ((is_in_D[i] && i != j) || i == 0) {
                            vars[cnt] = w[k][i][j]; coefs[cnt] = 1.0; cnt++;
                        }
                    }
                    vars[cnt] = u[k][j]; coefs[cnt] = -1.0; cnt++;
                    snprintf(name, sizeof(name), "c24_%d_%d", k, j);
                    SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, 0.0));
                    SCIP_CALL(SCIPaddCons(scip, cons));
                    SCIPreleaseCons(scip, &cons);
                }
            }
        }
    }


    // constrain 25: sum_(j in D) {y[k][i][j]} <= Q*u[k][i] for k in K, i in D
    {
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int i = 1; i <= numberofPoints; i++) {
                if (is_in_D[i]) {
                    cnt = 0;
                    for (int j = 1; j <= numberofPoints; j++) {
                        vars[cnt] = y[k][i][j]; coefs[cnt] = 1.0; cnt++;
                    }
                    vars[cnt] = u[k][i]; coefs[cnt] = -1.0*DRONE_CAPACITY; cnt++;
                    snprintf(name, sizeof(name), "c25_%d_%d", k, i);
                    SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, -SCIPinfinity(scip), 0.0));
                    SCIP_CALL(SCIPaddCons(scip, cons));
                    SCIPreleaseCons(scip, &cons);
                }
            }
        }
    }


    // constraint 26: sum_(j in D) {y[k][i][j]} >= u[k][i] for k in K, i in D
    {
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int i = 1; i <= numberofPoints; i++) {
                if (is_in_D[i]) {
                    cnt = 0;
                    for (int j = 1; j <= numberofPoints; j++) {
                        vars[cnt] = y[k][i][j]; coefs[cnt] = 1.0; cnt++;
                    }
                    vars[cnt] = u[k][i]; coefs[cnt] = -1.0; cnt++;
                    snprintf(name, sizeof(name), "c26_%d_%d", k, i);
                    SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, SCIPinfinity(scip)));
                    SCIP_CALL(SCIPaddCons(scip, cons));
                    SCIPreleaseCons(scip, &cons);
                }
            }
        }
    }

    // constraint 27: sum_(k in K, i in D){y[k][i][j]} + gamma[j] = z[j] for j in N
    {
        for (int j = 1; j <= numberofPoints; j++) {
            cnt = 0;
            for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                for (int i = 1; i <= numberofPoints; i++) {
                    if (is_in_D[i]) {
                        vars[cnt] = y[k][i][j]; coefs[cnt] = 1.0; cnt++;
                    }
                }
            }
            vars[cnt] = gamma[j]; coefs[cnt] = 1.0; cnt++;
            vars[cnt] = z[j]; coefs[cnt] = -1.0; cnt++;
            snprintf(name, sizeof(name), "c27_%d", j);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, 0.0));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
    }

    // constraint 28: sum_(k in K) {u[k][i]} <= z[i] for i in D
    {
        for (int i = 1; i < numberofPoints; i++) {
            if (is_in_D[i]) {
                cnt = 0;
                for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                    vars[cnt] = u[k][i]; coefs[cnt] = 1.0; cnt++;
                }
                vars[cnt] = z[i]; coefs[cnt] = -1.0; cnt++;
                snprintf(name, sizeof(name), "c28_%d", i);
                SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, -SCIPinfinity(scip), 0.0));
                SCIP_CALL(SCIPaddCons(scip, cons));
                SCIPreleaseCons(scip, &cons);
            }
        }
    }



    // constraint 29: T[j] >= T[i] + truck_time[i][j] - M*(1 - sum_(k in K){y[k][i][j]}) for i in N, j in D\{i}
    {
        for (int i = 1; i <= numberofPoints; i++) {
            for (int j = 1; j <= numberofPoints; j++) {
                if (is_in_D[j] && j != i) {
                    cnt = 0;
                    vars[cnt] = T[j]; coefs[cnt] = 1.0; cnt++;
                    vars[cnt] = T[i]; coefs[cnt] = -1.0; cnt++;

                    for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                        vars[cnt] = y[k][i][j]; coefs[cnt] = -1.0*M; cnt++;
                    }

                    snprintf(name, sizeof(name), "c29_%d_%d", i, j);
                    SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, truckTravelTime[i][j]-M, SCIPinfinity(scip)));
                    SCIP_CALL(SCIPaddCons(scip, cons));
                    SCIPreleaseCons(scip, &cons);

                }
            }
        }
    }

    // constraint 30: T[j] >= T[i] + truck_time[i][j] - M*(1-x[i][j]) for i in N || i == 0, (j in N\D, j != i) || j == n+1
    {
        for (int i = 0; i <= numberofPoints; i++) {
            for (int j = 1; j <= numberofPoints+1; j++) {
                if (!is_in_D[j] && j != i) {
                    cnt = 0;
                    vars[cnt] = T[j]; coefs[cnt] = 1.0; cnt++;
                    vars[cnt] = T[i]; coefs[cnt] = -1.0; cnt++;
                    vars[cnt] = x[i][j]; coefs[cnt] = -1.0*M; cnt++;
                    snprintf(name, sizeof(name), "c30_%d_%d", i, j);
                    SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, truckTravelTime[i][j]-M, SCIPinfinity(scip)));
                    SCIP_CALL(SCIPaddCons(scip, cons));
                    SCIPreleaseCons(scip, &cons);
                }
            }
        }
    }

    // constraint 31: T[j] >= T[i] + truck_time[i][j] + RESUPPLY_TIME*(sum_(k in K){u[k][j]} - M*(1-x[i][j]) for i in N || i == 0, j in D\{i}
    {
        for (int i = 0; i <= numberofPoints; i++) {
            for (int j = 1; j <= numberofPoints; j++) {
                if (is_in_D[j] && j != i) {
                    cnt = 0;
                    vars[cnt] = T[j]; coefs[cnt] = 1.0; cnt++;
                    vars[cnt] = T[i]; coefs[cnt] = -1.0; cnt++;
                    vars[cnt] = x[i][j]; coefs[cnt] = -1.0*M; cnt++;
                    for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                        vars[cnt] = u[k][j]; coefs[cnt] = -1.0*RESUPPLY_TIME; cnt++;
                    }
                    snprintf(name, sizeof(name), "c31_%d_%d", i, j);
                    SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, truckTravelTime[i][j]-M, SCIPinfinity(scip)));
                    SCIP_CALL(SCIPaddCons(scip, cons));
                    SCIPreleaseCons(scip, &cons);
                }
            }
        }
    }


    // constraint 32: T[j] >= T[i] + drone_time[i]+RESUPPLY_TIME+drone_time[j] - M*(1-sum_(k in K){w[k][i][j]})
    //                      for i in D, j in D\{i}
    {
        for (int i = 1; i <= numberofPoints; i++) {
            if (is_in_D[i]) {
                for (int j = 1; j <= numberofPoints; j++) {
                    if (is_in_D[j] && j != i) {
                        cnt = 0;
                        vars[cnt] = T[j]; coefs[cnt] = 1.0; cnt++;
                        vars[cnt] = T[i]; coefs[cnt] = -1.0; cnt++;
                        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                            vars[cnt] = w[k][i][j]; coefs[cnt] = -1.0*M; cnt++;
                        }
                        double lhs = droneTravelTime[i]+droneTravelTime[j]+RESUPPLY_TIME-M;
                        snprintf(name, sizeof(name), "c32_%d_%d", i, j);
                        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, lhs, SCIPinfinity(scip)));
                        SCIP_CALL(SCIPaddCons(scip, cons));
                        SCIPreleaseCons(scip, &cons);
                    }
                }
            }
        }

    }

    // constraint 33: T[i] >= (r[j] + drone_time[i] + RESUPPLY_TIME)*(sum_(k in K){y[k][i][j]}); for i in D, j in D
    {
        for (int i = 1; i <= numberofPoints; i++) {
            if (is_in_D[i]) {
                for (int j = 1; j <= numberofPoints; j++) {
                    if (is_in_D[j]) {
                        cnt = 0;
                        vars[cnt] = T[i]; coefs[cnt] = 1.0; cnt++;
                        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                            vars[cnt] = y[k][i][j]; coefs[cnt] = -1.0*(listofPoints[j].releaseTime+droneTravelTime[i]+RESUPPLY_TIME); cnt++;
                        }
                        snprintf(name, sizeof(name), "c33_%d_%d", i, j);
                        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, SCIPinfinity(scip)));
                        SCIP_CALL(SCIPaddCons(scip, cons));
                        SCIPreleaseCons(scip, &cons);
                    }
                }
            }
        }
    }

    // constraint 34: T[0] >= r[i]*gamma[i] for i in N
    {
        for (int i = 1; i <= numberofPoints; i++) {
            cnt = 0;
            vars[cnt] = T[0]; coefs[cnt] = 1.0; cnt++;
            vars[cnt] = gamma[i]; coefs[cnt] = -1.0*(listofPoints[i].releaseTime); cnt++;
            snprintf(name, sizeof(name), "c34_%d", i);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, SCIPinfinity(scip)));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
    }

    // constraint 35: T[n+1] <= H
    {
        cnt = 0;
        vars[cnt] = T[numberofPoints+1]; coefs[cnt] = 1.0; cnt++;
        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, "c35", cnt, vars, coefs, 0.0, H*1.0));
        SCIP_CALL(SCIPaddCons(scip, cons));
        SCIPreleaseCons(scip, &cons);
    }


    // extra constraint:

    // constraint 42: T[n+1] >= T[i] + truck_time[i][n+1] - M*(1-z[i]) for i in N
    {
        for (int i = 1; i <= numberofPoints; i++) {
            cnt = 0;
            vars[cnt] = T[numberofPoints+1]; coefs[cnt] = 1.0; cnt++;
            vars[cnt] = T[i]; coefs[cnt] = -1.0; cnt++;
            vars[cnt] = z[i]; coefs[cnt] = -1.0*M; cnt++;
            snprintf(name, sizeof(name), "c42_%d", i);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, truckTravelTime[i][numberofPoints+1]-M, SCIPinfinity(scip)));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
    }


    // constraint 43: T[i] >= r[i] + min(drone_time[i] + RESUPPLY_TIME, truck_time[0][i] for i in N
    {
        for (int i = 1; i <= numberofPoints; i++) {
            cnt = 0;
            vars[cnt] = T[i]; coefs[cnt] = 1.0; cnt++;
            snprintf(name, sizeof(name), "c43_%d", i);
            double lhs = 1.0*listofPoints[i].releaseTime + min(droneTravelTime[i]+1.0*RESUPPLY_TIME, truckTravelTime[0][i]);
            SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, lhs, SCIPinfinity(scip)));
            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIPreleaseCons(scip, &cons);
        }
    }


    // constraint 44: T[n+1] >= T[0] + sum_((i,j) in E){truck_time[i][j]*x[i][j]} + sum_(i in D){e[i]}
    {
        cnt = 0;
        vars[cnt] = T[numberofPoints+1]; coefs[cnt] = 1.0; cnt++;
        vars[cnt] = T[0]; coefs[cnt] = -1.0; cnt++;
        for (int i = 0; i <= numberofPoints; i++) {
            for (int j = 1; j <= numberofPoints+1; j++) {
                if (i == 0 && j == numberofPoints+1) continue; // this edge is not existed
                if (i != j) {
                    vars[cnt] = x[i][j]; coefs[cnt] = -1.0*truckTravelTime[i][j]; cnt++;
                }
            }
        }
        for (int i = 1; i <= numberofPoints; i++) {
            if (is_in_D[i]) {
                vars[cnt] = e[i]; coefs[cnt] = -1.0; cnt++;
            }
        }
        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, 0.0, SCIPinfinity(scip)));
        SCIP_CALL(SCIPaddCons(scip, cons));
        SCIPreleaseCons(scip, &cons);

    }

    // constraint 45: e[j] >= T[j] - (T[i] + truck_time[i][j]) - M*(1 - x[i][j]) for j in D, i in N\{j} || i == 0
    {
        for (int j = 1; j <= numberofPoints; j++) {
            if (is_in_D[j]) {
                for (int i = 0; i <= numberofPoints; i++) {
                    if (i != j) {
                        cnt = 0;
                        vars[cnt] = e[j]; coefs[cnt] = 1.0; cnt++;
                        vars[cnt] = T[j]; coefs[cnt] = -1.0; cnt++;
                        vars[cnt] = T[i]; coefs[cnt] = 1.0; cnt++;
                        vars[cnt] = x[i][j]; coefs[cnt] = -M*1.0; cnt++;
                        double lhs = -truckTravelTime[i][j] - M;
                        snprintf(name, sizeof(name), "c45_%d_%d", i, j);
                        SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, lhs, SCIPinfinity(scip)));
                        SCIP_CALL(SCIPaddCons(scip, cons));
                        SCIPreleaseCons(scip, &cons);
                    }
                }
            }
        }
    }


    // constraint 46: e[j] <= M*sum_(k in K){u[k][j]} for j in D
    {
        for (int j = 1; j <= numberofPoints; j++) {
            if (is_in_D[j]) {
                cnt = 0;
                vars[cnt] = e[j]; coefs[cnt] = 1.0; cnt++;
                for (int k = 0; k < NUMBER_OF_DRONES; k++) {
                    vars[cnt] = u[k][j]; coefs[cnt] = -1.0*M; cnt++;
                }
                snprintf(name, sizeof(name), "c46_%d", j);
                SCIP_CALL(SCIPcreateConsBasicLinear(scip, &cons, name, cnt, vars, coefs, -SCIPinfinity(scip), 0.0));
                SCIP_CALL(SCIPaddCons(scip, cons));
                SCIPreleaseCons(scip, &cons);
            }
        }
    }


    // solving
    SCIP_CALL( SCIPsolve(scip) );
    sol = SCIPgetBestSol(scip);
    if(sol) printf("Optimal delivered: %g\n", SCIPgetSolOrigObj(scip, sol));

    //releaseVar

    {

        for (int i = 0; i <= numberofPoints+1; i++) {
            for (int j = 0; j <= numberofPoints+1; j++) {
                if (i != j) {
                    SCIP_CALL(SCIPreleaseVar(scip, &x[i][j]));
                }
            }
        }
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int i = 0; i <= numberofPoints; i++) {
                if (is_in_D[i] || i == 0) SCIP_CALL(SCIPreleaseVar(scip, &u[k][i]));
            }
        }
        for(int i = 1; i <= numberofPoints; i++) {
            SCIP_CALL(SCIPreleaseVar(scip, &z[i]));
            SCIP_CALL(SCIPreleaseVar(scip, &gamma[i]));
        }

        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int i = 0; i < numberofPoints+2;i++) {
                if (is_in_D[i] || i==0 ) {
                    for (int j = 1; j < numberofPoints+2; j++) {
                        if (j != i && (is_in_D[j] || j == numberofPoints+1)) {
                            SCIP_CALL(SCIPreleaseVar(scip, &w[k][i][j]));
                        }
                    }
                }
            }
        }
        for (int k = 0; k < NUMBER_OF_DRONES; k++) {
            for (int i = 1; i <= numberofPoints; i++) {
                for (int j = 1; j <= numberofPoints; j++) {
                    SCIP_CALL(SCIPreleaseVar(scip, &y[k][i][j]));
                }
            }
        }
        for (int i = 0; i <= numberofPoints+1; i++) {
            SCIP_CALL(SCIPreleaseVar(scip, &T[i]));
            if (i != 0 && i != numberofPoints+1)
                SCIP_CALL(SCIPreleaseVar(scip, &e[i]));
        }
        SCIPfreeBufferArray(scip, &vars);
        SCIPfreeBufferArray(scip, &coefs);
        //SCIP_CALL(SCIPreleaseCons(scip, &cons));
    }

    SCIP_CALL( SCIPfree(&scip) );
}

