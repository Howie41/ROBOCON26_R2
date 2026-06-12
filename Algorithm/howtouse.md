# Algorithm 层

无锁数据结构和工具类，供 Module/BSP/APP 层使用。

## 模块列表

| 文件 | 功能 |
|------|------|
| lockfree_queue.hpp | MPSC 无锁队列 (CAS 原子操作)，用于中断→任务数据传递 |
| lockfree_ringbuffer.hpp | SPSC 无锁环形缓冲，用于 topic 发布-订阅 |
| double_buffer.hpp | 双缓冲，无锁读写分离 |
| function.hpp | 轻量级 std::function 替代 |

## 使用场景

- **lockfree_queue**: CAN 帧发送缓冲 (CanBus)，UART/USB 接收缓冲 (UartPort/UsbPort)
- **lockfree_ringbuffer**: Topic 系统订阅者队列 (每个订阅者独立 SPSC 环形缓冲)
- **double_buffer**: 预留，可用于传感器数据的无锁更新
- **function**: 回调存储 (如 CanBus 的 txService 回调)
