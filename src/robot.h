unsigned char TOF_data[32] = {0};   //store 2 TOF frames
unsigned char TOF_length = 16;
unsigned char TOF_header[3] {0x57,0x00,0xFF};
unsigned long TOF_system_time = 0;
unsigned long TOF_distance = 0;
unsigned char TOF_status = 0;
unsigned int TOF_signal = 0;
unsigned char TOF_check = 0;

unsigned char TOF_data1[32] = {0};   //store 2 TOF frames
unsigned char TOF_length1 = 16;
unsigned char TOF_header1[3] {0x57,0x00,0xFF};
unsigned long TOF_system_time1 = 0;
unsigned long TOF_distance1 = 0;
unsigned char TOF_status1 = 0;
unsigned int TOF_signal1 = 0;
unsigned char TOF_check1 = 0;

const int trigPin1 = 23;
const int echoPin1 = 22;

const int trigPin3 = 27;
const int echoPin3 = 26;

// Define constants for mathematical calculations
#define PI 3.1415926535897932384626433832795


long Kp = 1.5;  // Start with a lower Kp value
long Ki = 0.0;  // Start with Ki = 0
long Kd = 0.0;  // Start with Kd = 0

long integral = 0.0;
long previousError = 0.0;

int errors, turns;

long error;


double full_per_omega;

float Vx;
float Vy;
float omega;

double rollValue = 0;
double prevValue = 0;
double Right = 0;
double Left = 360;
double threshold = 180;
double delta_omega = 0;

double count_omega = 0;

int currentOmegaPosition = 0;


int button;

long duration_1;
int distance_1;

long duration_3;
int distance_3;

double imu_delta;

unsigned long receivedDistance = 0;

void ultr_sonic_1();
void ultr_sonic_3();
