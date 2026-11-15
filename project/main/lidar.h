
/*
 *   Header [lidar.h]
 */

#ifndef LIDAR_HEADER_GUARD
#define LIDAR_HEADER_GUARD

typedef struct {
  int distance_cm;
  int amplitude;
  float temp_c;
} lidar_data_t;

void lidar_init(void);
lidar_data_t lidar_read(void);

#endif /* LIDAR_HEADER_GUARD */
