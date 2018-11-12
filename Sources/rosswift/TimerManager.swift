
import Foundation

let g_timer_manager = InternalTimerManager()

func getInternalTimerManager() -> InternalTimerManager {
    return g_timer_manager
}

typealias InternalTimerManager = TimerManager


func initInternalTimerManager() {
    ROS_ERROR("initInternalTimerManager not implemented")
}

class TimerManager {

    func remove(timer_handle: Int32) {
        ROS_ERROR("\(#function) not implemented")
    }

}
