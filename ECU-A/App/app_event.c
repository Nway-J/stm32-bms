/*
 * 文件名称：app_event.c
 *
 * 模块作用：实现BMS状态命令的RAM环形队列。
 * Event_Push()负责写入命令，Event_Process()负责依次取出命令并交给状态机。
 *
 * 队列规则：
 * - head指向下一个待读取位置，tail指向下一个待写入位置；
 * - head == tail表示空；
 * - 为区分队列满和队列空，8个数组位置中最多保存7个事件；
 * - 队列满时保留旧事件，丢弃新事件；
 * - 当前实现只用于裸机主循环，不支持中断与主循环并发访问。
 *
 * 示例流程：
 * Event_Push(EVENT_FAULT) -> Event_Process()
 * -> BMS_EventHandle(EVENT_FAULT) -> BMS状态切换为BMS_FAULT。
 */
#include "stm32f10x.h"
#include "app_event.h"
#include "app_bms_state.h"

//队列容量
#define EVENT_QUEUE_SIZE    8
//事件缓冲区
static Event_t g_event_queue[EVENT_QUEUE_SIZE];
//读/写指针
static uint8_t g_event_head = 0;
static uint8_t g_event_tail = 0;

//初始化队列数据，读/写指针归0
void Event_Init()
{
	 g_event_head = 0;
   g_event_tail = 0;
}




// Event_Push — 压入一个事件到队列尾部
//
// 参数：event → 要压入的事件
//
// 工作流程：
//   1. 如果是EVENT_NONE，直接丢弃（不处理空事件）
//   2. 计算下一个写指针位置：(tail + 1) % SIZE
//   3. 如果下一个位置不等于head，说明队列没满：
//        → 把事件写入当前tail位置
//        → tail移动到下一个位置
//   4. 如果队列满了，直接丢弃这个事件（不覆盖旧数据）
//
// 当前只允许在裸机主循环上下文调用；并发访问需要临界区或RTOS队列保护。

void Event_Push(Event_t event)
{
	//丢弃事件
	if(event == EVENT_NONE)
        return;

	//计算下一个位置
	uint8_t next = (g_event_tail + 1) % EVENT_QUEUE_SIZE;
	
	//检查队列是否满
	if(next != g_event_head)
	{
		//没满，存入事件
		g_event_queue[g_event_tail] = event;
		//写指针前进
		g_event_tail = next;
		
	}
		
	//队列满了丢弃事件
}


// Event_Pop — 从队列头部取出一个事件
//
// 返回值：取出的事件，队列为空时返回EVENT_NONE
//
// 工作流程：
//   1. 如果head == tail，队列为空，返回EVENT_NONE
//   2. 从head位置取出事件
//   3. head移动到下一个位置
//   4. 返回取出的事件
//
// 在main循环中调用，不在中断中调用

Event_t Event_Pop(void)
{
	//检查队列是否空
	if(g_event_head == g_event_tail)
		return EVENT_NONE;
	
	//取出当前HEAD位置事件
	Event_t event = g_event_queue[g_event_head];
	
	//读指针继续前进
	g_event_head = (g_event_head + 1) % EVENT_QUEUE_SIZE;
	
	return event;
}



// Event_Available — 查询队列中是否有事件
// 返回值：0=队列空  非0=有事件

uint8_t Event_Available(void)
{
	//head != tail 说明队列不为空
	return (g_event_head != g_event_tail);
	
}



// Event_Process — 处理队列中所有待处理事件
//
// 循环取出队列中的每个事件，交给BMS_EventHandle处理
// 由调度器每20ms调用一次
//
// 工作流程：
//   while(队列不为空)
//   {
//       弹出一个事件
//       交给BMS_EventHandle()处理状态切换
//   }

void Event_Process(void)
{
	//循环队列内的所有事件
	while(Event_Available())
	{
		//弹出一个事件
		Event_t evt = Event_Pop();
		// 交给状态机模块处理
    // BMS_EventHandle会根据事件类型和当前状态
    // 决定是否切换状态
    BMS_EventHandle(evt);
	}
}
















