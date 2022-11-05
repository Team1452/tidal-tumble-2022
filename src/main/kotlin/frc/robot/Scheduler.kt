package frc.robot

class Scheduler {
    var tasks: MutableList<Pair<Int, () -> Unit>> = mutableListOf()

    fun scheduleIn(delay: Int, task: () -> Unit) {
        tasks.add(Pair(delay, task))
    }

    fun tick() {
        tasks.forEachIndexed { index, task ->
            tasks[index] = Pair(task.first - 1, task.second)
            if (task.first - 1 <= 0) {
                task.second.invoke()
                tasks.removeAt(index)
            }
        }
    }
}