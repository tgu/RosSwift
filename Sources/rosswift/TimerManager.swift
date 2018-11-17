import Foundation

let gTimerManager = InternalTimerManager()

func getInternalTimerManager() -> InternalTimerManager {
    return gTimerManager
}

typealias InternalTimerManager = TimerManager

func initInternalTimerManager() {
    ROS_ERROR("initInternalTimerManager not implemented")
}

final class TimerManager {

    func remove(timerHandle: Int32) {
        ROS_ERROR("\(#function) not implemented")
    }

}
