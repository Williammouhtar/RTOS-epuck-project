#define SPEED                   600
#define DEGREE_TO_STEPS         3.56
#define SMALL_STEP              150
#define BIG_STEP                450
#define EPSILON_ANGLE           3
#define SMALL_SLEEP_MS           5

void turn_left(uint16_t angle);
void turn_right(uint16_t angle);
void move_forward_extra(int32_t position);
void move_backward_extra(int32_t position);