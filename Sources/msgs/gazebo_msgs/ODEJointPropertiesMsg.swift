import Foundation
import StdMsgs
import RosTime


extension gazebo_msgs {
/// access to low level joint properties, change these at your own risk
public struct ODEJointProperties: Message {
public static var md5sum: String = "1b744c32a920af979f53afe2f9c3511f"
public static var datatype = "gazebo_msgs/ODEJointProperties"
public static var definition = """
# access to low level joint properties, change these at your own risk
float64[] damping             # joint damping
float64[] hiStop              # joint limit
float64[] loStop              # joint limit
float64[] erp                 # set joint erp
float64[] cfm                 # set joint cfm
float64[] stop_erp            # set joint erp for joint limit "contact" joint
float64[] stop_cfm            # set joint cfm for joint limit "contact" joint
float64[] fudge_factor        # joint fudge_factor applied at limits, see ODE manual for info.
float64[] fmax                # ode joint param fmax
float64[] vel                 # ode joint param vel
"""
public static var hasHeader = false

public var damping: [Float64]
public var hiStop: [Float64]
public var loStop: [Float64]
public var erp: [Float64]
public var cfm: [Float64]
public var stop_erp: [Float64]
public var stop_cfm: [Float64]
public var fudge_factor: [Float64]
public var fmax: [Float64]
public var vel: [Float64]

public init(damping: [Float64], hiStop: [Float64], loStop: [Float64], erp: [Float64], cfm: [Float64], stop_erp: [Float64], stop_cfm: [Float64], fudge_factor: [Float64], fmax: [Float64], vel: [Float64]) {
self.damping = damping
self.hiStop = hiStop
self.loStop = loStop
self.erp = erp
self.cfm = cfm
self.stop_erp = stop_erp
self.stop_cfm = stop_cfm
self.fudge_factor = fudge_factor
self.fmax = fmax
self.vel = vel
}

public init() {
    damping = [Float64]()
hiStop = [Float64]()
loStop = [Float64]()
erp = [Float64]()
cfm = [Float64]()
stop_erp = [Float64]()
stop_cfm = [Float64]()
fudge_factor = [Float64]()
fmax = [Float64]()
vel = [Float64]()
}

}
}
