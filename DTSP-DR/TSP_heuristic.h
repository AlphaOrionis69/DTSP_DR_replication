#include <stdio.h>
#include <stdlib.h>
#include <time.h>



int num_nodes;
double **dist_matrix;

// Compute total length of the given tour over the distance matrix.
double tour_length(int *tour, int n, double **dist) {
    double length = 0;
    for (int i = 0; i < n - 1; i++) {
        length += dist[tour[i]][tour[i+1]];
    }
    // add return edge if treating as cycle; omitted if fixed endpoints
    return length;
}

// Reverse the tour segment between indices i and j (inclusive).
void two_opt_swap(int *tour, int i, int j) {
    while (i < j) {
        int tmp = tour[i];
        tour[i] = tour[j];
        tour[j] = tmp;
        i++;
        j--;
    }
}

// Perform 2-opt improvement until no further improvement is possible.
void two_opt(int *tour, int n, double **dist) {
    bool improved = true;
    while (improved) {
        improved = false;
        double best_delta = 0.0;
        int best_i = -1, best_j = -1;
        //double current_length = tour_length(tour, n, dist);

        // i from 1 to n-3, j from i+1 to n-2: keep endpoints fixed
        for (int i = 1; i < n - 2; i++) {
            for (int j = i + 1; j < n - 1; j++) {
                // Calculate change in length if we reverse between i and j
                int a = tour[i - 1];
                int b = tour[i];
                int c = tour[j];
                int d = tour[j + 1];
                double delta = dist[a][c] + dist[b][d] - dist[a][b] - dist[c][d];
                if (delta < best_delta) {
                    best_delta = delta;
                    best_i = i;
                    best_j = j;
                    improved = true;
                }
            }
        }
        if (improved && best_i >= 0) {
            two_opt_swap(tour, best_i, best_j);
        }
    }
}

void TSP_heuristic(double** travelTime, int total, int* tour) {
    num_nodes = total;

    // Allocate distance matrix
    dist_matrix = malloc(num_nodes * sizeof(*dist_matrix));
    for (int i = 0; i < num_nodes; i++) {
        dist_matrix[i] = malloc(num_nodes * sizeof(double));
        for (int j = 0; j < num_nodes; j++) {
            dist_matrix[i][j] = travelTime[i][j];
        }
    }

    // Initialize a simple tour: 0,1,2,...,n-1
    for (int i = 0; i < num_nodes; i++) {
        tour[i] = i;
    }

    // Apply 2-opt heuristic
    two_opt(tour, num_nodes, dist_matrix);

    // Print the final tour
    for (int i = 0; i < num_nodes; i++) {
        printf("%d ", tour[i]);
    }
    printf("\n%lf", tour_length(tour, num_nodes, dist_matrix));
    printf("\n");

    // Clean up
    for (int i = 0; i < num_nodes; i++) free(dist_matrix[i]);
    free(dist_matrix);
    // free(tour);
    return;
}




    // Genetic Algorithm
    /*
    #define MAXNODE 105
    #define POP_SIZE 100
    #define MAX_GEN 200
    #define CROSSOVER_RATE 0.8
    #define MUTATION_RATE 0.2
    // Individual representation: a tour (permutation of nodes) keeping first and last fixed

    typedef struct {
        int *tour;
        double fitness;
    } Individual;

    void printTour(Individual* ind) {
        for (int i = 0; i < num_nodes; i++) {
            printf("%d ", ind->tour[i]);
        }
        printf("\n");
    }

    // Allocate a new individual with fixed endpoints
    Individual create_individual() {
        Individual ind;
        ind.tour = (int*)malloc(num_nodes * sizeof(int));
        // Fix endpoints
        ind.tour[0] = 0;
        ind.tour[num_nodes - 1] = num_nodes - 1;
        // Fill middle nodes
        int mid_count = num_nodes - 2;
        int *middle = (int*)malloc(mid_count * sizeof(int));
        for (int i = 0; i < mid_count; i++) {
            middle[i] = i + 1;
        }
        // Shuffle middle
        for (int i = mid_count - 1; i > 0; i--) {
            int j = rand() % (i + 1);
            int tmp = middle[i];
            middle[i] = middle[j];
            middle[j] = tmp;
        }
        // Assign middle to tour
        for (int i = 1; i <= mid_count; i++) {
            ind.tour[i] = middle[i - 1];
        }
        free(middle);
        ind.fitness = 0;
        return ind;
    }


    // Compute fitness: total path length (lower is better)
    void compute_fitness(Individual *ind) {
        double length = 0.0;

        for (int i = 0; i < num_nodes - 1; i++) {
            length += dist_matrix[ind->tour[i]][ind->tour[i+1]];
        }

        ind->fitness = length;
    }

    // Compare two individuals by fitness (for qsort)
    int cmp_ind(const void *a, const void *b) {
        return ((Individual *)a)->fitness - ((Individual *)b)->fitness;
    }

    // Tournament selection
    Individual tournament_select(Individual *pop) {
        int k = 5; // tournament size
        Individual best = pop[rand() % POP_SIZE];
        for (int i = 1; i < k; i++) {
            Individual challenger = pop[rand() % POP_SIZE];
            if (challenger.fitness < best.fitness) {
                best = challenger;
            }
        }
        // Copy individual
        Individual copy;
        copy.tour = malloc(num_nodes * sizeof(int));
        for (int i = 0; i < num_nodes; i++) copy.tour[i] = best.tour[i];
        copy.fitness = best.fitness;
        return copy;
    }

    // Ordered Crossover (OX) for middle segment only
    void crossover(Individual parent1, Individual parent2, Individual *child1, Individual *child2) {
        int start = rand() % (num_nodes - 2) + 1;      // between 1 and num_nodes-2
        int end = rand() % (num_nodes - 2) + 1;
        if (start > end) { int tmp = start; start = end; end = tmp; }
        // Initialize children
        child1->tour[0] = 0;
        child2->tour[0] = 0;
        child1->tour[num_nodes - 1] = num_nodes - 1;
        child2->tour[num_nodes - 1] = num_nodes - 1;
        for (int i = 1; i < num_nodes - 1; i++) {
            child1->tour[i] = child2->tour[i] = -1;
        }
        // Copy segment
        for (int i = start; i <= end; i++) {
            child1->tour[i] = parent1.tour[i];
            child2->tour[i] = parent2.tour[i];
        }
        // Fill remaining middle
        int idx1 = (end + 1 <= num_nodes - 2) ? end + 1 : 1;
        int idx2 = idx1;
        int mid_count = num_nodes-2;
        // assume we got a genes like: [0, 1, .., start, .., end, end+1, .., n-1],
        // then we loop from end+1 to n-2 then back to 1 and go to start-1
        for (int i = 0; i < mid_count; i++) {
            int gene2 = parent2.tour[(end + 1 + i) % (num_nodes - 2)+1];
            int gene1 = parent1.tour[(end + 1 + i) % (num_nodes - 2)+1];
            // child1
            int found = 0;
            for (int j = start; j <= end; j++) if (child1->tour[j] == gene2) { found = 1; break; }
            if (!found && gene2 != 0 && gene2 != num_nodes-1) {
                child1->tour[idx1] = gene2;
                idx1 = (idx1 < num_nodes - 2) ? idx1 + 1 : 1;
            }
            // child2
            found = 0;
            for (int j = start; j <= end; j++) if (child2->tour[j] == gene1) { found = 1; break; }
            if (!found && gene1 != 0 && gene1 != num_nodes-1) {
                child2->tour[idx2] = gene1;
                idx2 = (idx2 < num_nodes - 2) ? idx2 + 1 : 1;
            }
        }
    }

    // Swap mutation (only in middle segment)
    void mutate(Individual *ind) {
        int i = rand() % (num_nodes - 2) + 1;
        int j = rand() % (num_nodes - 2) + 1;
        int tmp = ind->tour[i];
        ind->tour[i] = ind->tour[j];
        ind->tour[j] = tmp;
    }
    // use this function to create a TSP tour
    void TSP_heuristic(double** travelTime, int numberofNodes, int* final_tour) {
        srand(time(NULL));

        num_nodes = numberofNodes;
        dist_matrix = (double**)malloc(num_nodes*sizeof(double*));
        for (int i = 0; i < num_nodes; i++) {
            dist_matrix[i] = (double*)malloc(num_nodes * sizeof(double));
            for (int j = 0; j < num_nodes; j++) {
                dist_matrix[i][j] = travelTime[i][j];
            }
        }
        //printf("HI\n");
        // Initialize population
        Individual population[POP_SIZE];
        for (int i = 0; i < POP_SIZE; i++) {
            population[i] = create_individual();
            compute_fitness(&population[i]);

        }

        Individual new_pop[POP_SIZE];
        //printf("HI\n");
        // GA main loop
        for (int gen = 0; gen < MAX_GEN; gen++) {
            // Elitism: keep best 2
            qsort(population, POP_SIZE, sizeof(Individual), cmp_ind);
            new_pop[0] = population[0];
            new_pop[1] = population[1];
            int idx = 2;
            //printf("HI, %d\n", gen);
            while (idx < POP_SIZE) {
                Individual p1 = tournament_select(population);
                Individual p2 = tournament_select(population);
                //printf("   %d\n", idx);
                Individual c1, c2;
                c1.tour = (int*)malloc(num_nodes * sizeof(int));
                c2.tour = (int*)malloc(num_nodes * sizeof(int));

                if ((double)rand() / RAND_MAX < CROSSOVER_RATE) {
                    crossover(p1, p2, &c1, &c2);
                } else {
                    for (int i = 0; i < num_nodes; i++) {
                        c1.tour[i] = p1.tour[i];
                        c2.tour[i] = p2.tour[i];
                    }
                }
                printTour(&c1);
                printTour(&c2);
                //printf("   %d\n", idx);
                if ((double)rand() / RAND_MAX < MUTATION_RATE) mutate(&c1);
                if ((double)rand() / RAND_MAX < MUTATION_RATE) mutate(&c2);
                //printf("   %d\n", idx);
                compute_fitness(&c1);
                compute_fitness(&c2);
                //printf("   %d\n", idx);
                new_pop[idx++] = c1;
                if (idx < POP_SIZE) new_pop[idx++] = c2;
                //free(p1.tour);
                //free(p2.tour);
            }
            printf("HIHIHIHI\n");
            for (int i = 0; i < POP_SIZE; i++) {
                //printf("BRUH, %d\n", i);

                //free(population[i].tour);
                population[i] = new_pop[i];
            }
        }

        qsort(population, POP_SIZE, sizeof(Individual), cmp_ind);
        printf("Best tour length: %lf\n", population[0].fitness);
        printf("Tour: ");
        for (int i = 0; i < num_nodes; i++) printf("%d ", population[0].tour[i]);
        printf("\n");


        // copy the result to the final tour
        for (int i = 0; i < num_nodes; i++) {
            final_tour[i] = population[0].tour[i];
        }

        for (int i = 0; i < num_nodes; i++) free(dist_matrix[i]);
        free(dist_matrix);
        for (int i = 0; i < POP_SIZE; i++) free(population[i].tour);
        return;
    }
    */




