
#include "Fifo.h"
/********************************************************************************************************
Function Name: System_Init  
Author       : ZFY
Date         : 2025-01-05
Description  :
Outputs      : void
Notes        : 
********************************************************************************************************/


#include <stdio.h>
#include <stdlib.h>



// 定义结构体表示FIFO队列
typedef struct {
    NSM_77 *data;    // 存储队列元素的动态数组指针，元素类型为Data结构体
    int front;    // 队头索引
    int rear;     // 队尾索引
    int capacity; // 队列容量
} FifoQueue;

// 初始化FIFO队列
FifoQueue* initFifoQueue(int capacity) {
    FifoQueue *queue = (FifoQueue *)malloc(sizeof(FifoQueue));
    if (queue == NULL) {
//        printf("内存分配失败！\n");
        return NULL;
    }
    queue->data = (NSM_77 *)malloc(capacity * sizeof(NSM_77));
    if (queue->data == NULL) {
//        printf("内存分配失败！\n");
        free(queue);
        return NULL;
    }
    queue->front = 0;
    queue->rear = -1;
    queue->capacity = capacity;
    return queue;
}

// 判断队列是否为空
int isEmpty(FifoQueue *queue) {
    return (queue->rear == -1 && queue->front == 0);
}

// 判断队列是否已满
int isFull(FifoQueue *queue) {
    return ((queue->rear + 1) % queue->capacity == queue->front);
}

// 入队操作，满了则自动删除队头数据腾出空间
void enqueue(FifoQueue *queue, NSM_77 element) {
    if (isFull(queue)) {
        // 队满，删除队头元素，更新队头索引
        queue->front = (queue->front + 1) % queue->capacity;
    }
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->data[queue->rear] = element;
}

// 出队操作
NSM_77 dequeue(FifoQueue *queue) {
    if (isEmpty(queue)) {
        NSM_77 empty_data = { -1 };  // 返回一个表示空的结构体，这里简单用num为 -1 表示
//        printf("队列为空，无法出队！\n");
        return empty_data;
    }
    NSM_77 element = queue->data[queue->front];
    if (queue->front == queue->rear) {
        // 只剩一个元素时，出队后队列置空
        queue->front = 0;
        queue->rear = -1;
    } else {
        queue->front = (queue->front + 1) % queue->capacity;
    }
    return element;
}

// 获取队列中的最小元素（根据结构体中num成员比较大小）
NSM_77 getMinElement(FifoQueue *queue) {
    if (isEmpty(queue)) {
        Data empty_data = { -1 };  // 返回一个表示空的结构体，这里简单用num为 -1 表示
        printf("队列为空，无最小元素！\n");
        return empty_data;
    }
    Data min = queue->data[queue->front];
    int i;
    int index = queue->front;
    for (i = 0; i < (queue->rear >= queue->front? queue->rear - queue->front + 1 : queue->capacity - queue->front + queue->rear + 1); i++) {
        index = (index + 1) % queue->capacity;
        if (queue->data[index].num < min.num) {
            min = queue->data[index];
        }
    }
    return min;
}

// 释放队列内存
void freeFifoQueue(FifoQueue *queue) {
    free(queue->data);
    free(queue);
}

int main() {
    FifoQueue *queue = initFifoQueue(5);  // 初始化一个容量为5的队列

    Data data1 = { 10 };
    Data data2 = { 5 };
    Data data3 = { 8 };
    Data data4 = { 3 };
    Data data5 = { 6 };
    Data data6 = { 2 };  // 用于在队列满后插入测试

    enqueue(queue, data1);
    enqueue(queue, data2);
    enqueue(queue, data3);
    enqueue(queue, data4);
    enqueue(queue, data5);
    enqueue(queue, data6);  // 此时队列已满，插入此元素会自动删除队头元素data1

    Data min_data = getMinElement(queue);
    if (min_data.num!= -1) {
        printf("当前队列中的最小元素是: %d\n", min_data.num);
    }

    Data out_data = dequeue(queue);
    if (out_data.num!= -1) {
        printf("出队一个元素后，当前队列中的最小元素是: %d\n", getMinElement(queue).num);
    }

    freeFifoQueue(queue);

    return 0;
}




















