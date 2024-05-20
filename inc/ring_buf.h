#define BUF_SIZE 10

typedef struct {
    float buffer[BUF_SIZE];
    int curr_index;
} ring_buf;

void insert_val(float accel, ring_buf* b);
float get_rolling_avg(ring_buf* b);