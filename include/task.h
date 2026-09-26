#include "vex.h"
#include <functional>
#include <queue>
#include <thread>
namespace Task
{
using task = std::function<void()>;
struct asynctaskqueue
{
  private:
    std::queue<task> list;
    vex::thread t;
    static int _taskcycle(asynctaskqueue *q)
    {
        while (true)
        {
            if (q->list.empty())
            {
                vex::wait(50, vex::msec);
                continue;
            }

            task current = q->list.front();
            current();
            q->list.pop();
        }
        return 0;
    }

    static int taskcycle(void *param)
    {
        asynctaskqueue *ptr = static_cast<asynctaskqueue *>(param);
        _taskcycle(ptr);
    }

  public:
    asynctaskqueue()
    {
        t = vex::thread(taskcycle, this);
    }

    void emptylist();
    void push(task t);
    void interrupt();
    void wait();
};

} // namespace Task