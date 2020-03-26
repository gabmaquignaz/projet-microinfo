#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
float get_hor_pos(void);
void process_image_start(void);
void line_position (uint8_t* image, uint16_t size);

#endif /* PROCESS_IMAGE_H */
