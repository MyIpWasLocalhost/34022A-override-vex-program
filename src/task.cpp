#include "task.h"
#include "vex.h"
namespace Task
{
void asynctaskqueue::emptylist()
{
    while (!list.empty())
        list.pop();
}
void asynctaskqueue::push(task t)
{
    list.push(t);
}

void asynctaskqueue::interrupt()
{
    t.interrupt();
    emptylist();
    t = vex::thread(taskcycle, this);
}

void asynctaskqueue::wait()
{
    while (!list.empty())
    {
        vex::wait(50, vex::msec);
    }
}
} // namespace Task