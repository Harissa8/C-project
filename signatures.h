#define MAX_LINE_LENGTH 100
#define MAX_DRONES 70
#define MAX_NEIGHBORS 10
#define Vmax 40
#define POSITION_THRESHOLD 0.1
typedef struct Drone {
    int id;
    float x, y, z;
    float coverage;
    float vx, vy, vz; 
    float communication_range;
    float camera_resolution;
    int neighbors[MAX_NEIGHBORS];
    int neighbor_count;
    int is_active; 
    //double battery_Level;
} Drone;

typedef struct Image
{
 float xMin,xMax,yMin,yMax;
 float width,height;

}Image;



int is_position_occupied(float x, float y, float z, int moving_drone_id);  
void voronoi_adapt_all_drones(int moved_drone_id,Image*img1);  
void avoid_collisions(); 
float random_float(float min, float max);  
float calculate_distance(Drone* d1, Drone* d2);
float go_to(float x, float y, float z, int id,Image*img1);
float* speed(float vx, float vy, float vz, int id, float delta_t,Image*img1);
float calculate_coverage(float altitude);
void spread_drones(int num_drones, float xmin, float ymin, float xmax, float ymax, float comm_range, float cam_res,Image Im1);
int within_communication_range(Drone* d1, Drone* d2);
void apdate_neighbors();
void adjust_neighbors_for_destroyed_drone(int destroyed_drone_id);
void check_and_adapt();
void print_drone_infos();
void calculate_centroid(float* centroid_x, float* centroid_y, float* centroid_z);
void zoom_out(float z,int id,Image*img1);
void stretch(float x,float y,int id,Image*img1);
