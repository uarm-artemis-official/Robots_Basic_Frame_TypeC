#include "message_center.h"


static Publisher_t message_center = {
    .topic_name = "Message_Manager",
    .next_sub = NULL,
    .next_topic_node = NULL};

static void check_name_length(char *name)
{
    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) >= MAX_TOPIC_NAME_LEN)
    {
    	// Name is too long error.
        while (1);
    }
}

static void match_lengths(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
    	// Lengths do not match error.
        while (1);
    }
}

Publisher_t *register_pub(char *name, uint8_t data_len)
{
    check_name_length(name);
    Publisher_t *node = &message_center;
    while (node->next_topic_node)
    {
        node = node->next_topic_node;
        if (strcmp(node->topic_name, name) == 0)
        {
            match_lengths(data_len, node->data_len);
            node->pub_registered_flag = 1;
            return node;
        }
    }

    node->next_topic_node = (Publisher_t *)malloc(sizeof(Publisher_t));
    memset(node->next_topic_node, 0, sizeof(Publisher_t));
    node->next_topic_node->data_len = data_len;
    strcpy(node->next_topic_node->topic_name, name);
    node->pub_registered_flag = 1;
    return node->next_topic_node;
}

Subscriber_t *register_sub(char *name, uint8_t data_len) {
	// Not guaranteed that publishers are registered before subscribers
	// register publisher before trying to register subscriber.
    Publisher_t *pub = register_pub(name, data_len);

    Subscriber_t *ret = (Subscriber_t *)malloc(sizeof(Subscriber_t));
    memset(ret, 0, sizeof(Subscriber_t));

    ret->data_len = data_len;
    for (size_t i = 0; i < MAX_QUEUE_SIZE; ++i) {
        ret->queue[i] = malloc(data_len);
    }

    if (pub->next_sub == NULL)
    {
        pub->next_sub = ret;
        return ret;
    }

    Subscriber_t *sub = pub->next_sub;
    while (sub->next_sub) {
        sub = sub->next_sub;
    }
    sub->next_sub = ret;
    return ret;
}


uint8_t get_message(Subscriber_t *sub, void *rx_ptr)
{
    if (sub->queue_size == 0)
    {
        return 0;
    }
    memcpy(rx_ptr, sub->queue[sub->front_idx], sub->data_len);
    sub->front_idx = (sub->front_idx++) % MAX_QUEUE_SIZE;
    sub->queue_size--;
    return 1;
}


uint8_t pub_message(Publisher_t *pub, void *tx_ptr)
{
    static Subscriber_t *iter;
    iter = pub->next_sub;

    while (iter) {
        if (iter->queue_size == MAX_QUEUE_SIZE) {
        	iter->front_idx = (iter->front_idx + 1) % MAX_QUEUE_SIZE;
            iter->queue_size--;
        }

        memcpy(iter->queue[iter->back_idx], tx_ptr, pub->data_len);
        iter->back_idx = (iter->back_idx + 1) % MAX_QUEUE_SIZE;
        iter->queue_size++;

        iter = iter->next_sub;
    }
    return 1;
}
