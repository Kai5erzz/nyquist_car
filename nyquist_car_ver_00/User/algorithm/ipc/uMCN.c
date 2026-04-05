/**
 * @file uMCN.c
 * @brief uMCN (micro Message Communication Network) implementation
 */

#include "uMCN.h"
#include <string.h>

static McnList __mcn_list = { .hub = NULL, .next = NULL };

void mcn_node_clear(McnNode_t node_t)
{
    MCN_ASSERT(node_t != NULL);
    if (node_t == NULL) return;

    MCN_ENTER_CRITICAL;
    node_t->renewal = 0;
    MCN_EXIT_CRITICAL;
}

void mcn_suspend(McnHub_t hub) { hub->suspend = 1; }
void mcn_resume(McnHub_t hub) { hub->suspend = 0; }
McnList_t mcn_get_list(void) { return &__mcn_list; }

McnHub_t mcn_iterate(McnList_t* ite)
{
    McnHub_t hub;
    McnList_t node = *ite;
    if (node == NULL) return NULL;
    hub = node->hub;
    *ite = node->next;
    return hub;
}

bool mcn_poll(McnNode_t node_t)
{
    bool renewal;
    MCN_ASSERT(node_t != NULL);
    MCN_ENTER_CRITICAL;
    renewal = node_t->renewal;
    MCN_EXIT_CRITICAL;
    return renewal;
}

bool mcn_wait(McnNode_t node_t, int32_t timeout)
{
    MCN_ASSERT(node_t != NULL);
    MCN_ASSERT(node_t->event != NULL);
    return MCN_WAIT_EVENT(node_t->event, timeout) == 0 ? true : false;
}

fmt_err_t mcn_copy(McnHub_t hub, McnNode_t node_t, void* buffer)
{
    MCN_ASSERT(hub != NULL);
    MCN_ASSERT(node_t != NULL);
    MCN_ASSERT(buffer != NULL);
    if (hub->pdata == NULL) return FMT_ERROR;
    if (!hub->published) return FMT_ENOTHANDLE;

    MCN_ENTER_CRITICAL;
    memcpy(buffer, hub->pdata, hub->obj_size);
    node_t->renewal = 0;
    MCN_EXIT_CRITICAL;
    return FMT_EOK;
}

fmt_err_t mcn_copy_from_hub(McnHub_t hub, void* buffer)
{
    MCN_ASSERT(hub != NULL);
    MCN_ASSERT(buffer != NULL);
    if (hub->pdata == NULL) return FMT_ERROR;
    if (!hub->published) return FMT_ENOTHANDLE;

    MCN_ENTER_CRITICAL;
    memcpy(buffer, hub->pdata, hub->obj_size);
    MCN_EXIT_CRITICAL;
    return FMT_EOK;
}

fmt_err_t mcn_advertise(McnHub_t hub, int (*echo)(void* parameter))
{
    void* pdata;
    void* next;
    MCN_ASSERT(hub != NULL);
    if (hub->pdata != NULL) return FMT_ENOTHANDLE;

    pdata = MCN_MALLOC(hub->obj_size);
    if (pdata == NULL) return FMT_ENOMEM;
    memset(pdata, 0, hub->obj_size);

    next = MCN_MALLOC(sizeof(McnList));
    if (next == NULL) {
        MCN_FREE(pdata);
        return FMT_ENOMEM;
    }

    MCN_ENTER_CRITICAL;
    hub->pdata = pdata;
    hub->echo = echo;
    McnList_t cp = &__mcn_list;
    while (cp->next != NULL) cp = cp->next;
    if (cp->hub != NULL) {
        cp->next = (McnList_t)next;
        cp = cp->next;
    }
    cp->hub = hub;
    cp->next = NULL;
    memset(hub->freq_est_window, 0, 2 * MCN_FREQ_EST_WINDOW_LEN);
    hub->window_index = 0;
    MCN_EXIT_CRITICAL;

    return FMT_EOK;
}

McnNode_t mcn_subscribe(McnHub_t hub, MCN_EVENT_HANDLE event, void (*pub_cb)(void* parameter))
{
    MCN_ASSERT(hub != NULL);
    if (hub->link_num >= MCN_MAX_LINK_NUM) return NULL;
    McnNode_t node = (McnNode_t)MCN_MALLOC(sizeof(McnNode));
    if (node == NULL) return NULL;

    node->renewal = 0;
    node->event = event;
    node->pub_cb = pub_cb;
    node->next = NULL;

    MCN_ENTER_CRITICAL;
    if (hub->link_tail == NULL) {
        hub->link_head = hub->link_tail = node;
    } else {
        hub->link_tail->next = node;
        hub->link_tail = node;
    }
    hub->link_num++;
    MCN_EXIT_CRITICAL;

    if (hub->published) {
        node->renewal = 1;
        if (node->pub_cb) node->pub_cb(hub->pdata);
    }
    return node;
}

fmt_err_t mcn_unsubscribe(McnHub_t hub, McnNode_t node)
{
    MCN_ASSERT(hub != NULL);
    MCN_ASSERT(node != NULL);
    McnNode_t cur_node = hub->link_head;
    McnNode_t pre_node = NULL;

    while (cur_node != NULL) {
        if (cur_node == node) break;
        pre_node = cur_node;
        cur_node = cur_node->next;
    }
    if (cur_node == NULL) return FMT_EEMPTY;

    MCN_ENTER_CRITICAL;
    if (hub->link_num == 1) {
        hub->link_head = hub->link_tail = NULL;
    } else {
        if (cur_node == hub->link_head) hub->link_head = cur_node->next;
        else if (cur_node == hub->link_tail) {
            if (pre_node) pre_node->next = NULL;
            hub->link_tail = pre_node;
        } else pre_node->next = cur_node->next;
    }
    hub->link_num--;
    MCN_EXIT_CRITICAL;
    MCN_FREE(cur_node);
    return FMT_EOK;
}

fmt_err_t mcn_publish(McnHub_t hub, const void* data)
{
    MCN_ASSERT(hub != NULL);
    MCN_ASSERT(data != NULL);
    if (hub->pdata == NULL) return FMT_ERROR;
    if (hub->suspend) return FMT_ENOTHANDLE;

    hub->freq_est_window[hub->window_index]++;
    MCN_ENTER_CRITICAL;
    memcpy(hub->pdata, data, hub->obj_size);
    McnNode_t node = hub->link_head;
    while (node != NULL) {
        node->renewal = 1;
        if (node->event) MCN_SEND_EVENT(node->event);
        node = node->next;
    }
    hub->published = 1;
    MCN_EXIT_CRITICAL;

    node = hub->link_head;
    while (node != NULL) {
        if (node->pub_cb != NULL) node->pub_cb(hub->pdata);
        node = node->next;
    }
    return FMT_EOK;
}

fmt_err_t mcn_init(void) { return FMT_EOK; }