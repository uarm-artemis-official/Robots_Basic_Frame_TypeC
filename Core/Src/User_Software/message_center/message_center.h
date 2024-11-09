/**

 */

#ifndef PUBSUB_H
#define PUBSUB_H

#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "maths.h"

#define MAX_TOPIC_NAME_LEN 32
#define MAX_QUEUE_SIZE 1

typedef struct Subscriber_t
{
    void *queue[MAX_QUEUE_SIZE];
    uint8_t data_len;
    uint8_t front_idx;
    uint8_t back_idx;
    uint8_t queue_size;

    struct Subscriber_t *next_sub;
} Subscriber_t;


typedef struct Publisher_t
{

    char topic_name[MAX_TOPIC_NAME_LEN + 1];
    uint8_t data_len;

    Subscriber_t *next_sub;

    struct Publisher_t *next_topic_node;
    uint8_t pub_registered_flag;
} Publisher_t;

Subscriber_t *register_sub(char *name, uint8_t data_len);
Publisher_t *register_pub(char *name, uint8_t data_len);
uint8_t get_message(Subscriber_t *sub, void *data_ptr);
uint8_t pub_message(Publisher_t *pub, void *data_ptr);


#endif // !PUBSUB_H
