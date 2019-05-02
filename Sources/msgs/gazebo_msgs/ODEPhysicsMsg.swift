import Foundation
import StdMsgs
import RosTime


extension gazebo_msgs {

public struct ODEPhysics: Message {
public static var md5sum: String = "667d56ddbd547918c32d1934503dc335"
public static var datatype = "gazebo_msgs/ODEPhysics"
public static var definition = """
bool auto_disable_bodies           # enable auto disabling of bodies, default false
uint32 sor_pgs_precon_iters        # preconditioning inner iterations when uisng projected Gauss Seidel
uint32 sor_pgs_iters               # inner iterations when uisng projected Gauss Seidel
float64 sor_pgs_w                  # relaxation parameter when using projected Gauss Seidel, 1 = no relaxation
float64 sor_pgs_rms_error_tol      # rms error tolerance before stopping inner iterations
float64 contact_surface_layer      # contact "dead-band" width
float64 contact_max_correcting_vel # contact maximum correction velocity
float64 cfm                        # global constraint force mixing
float64 erp                        # global error reduction parameter
uint32 max_contacts                # maximum contact joints between two geoms
"""
public static var hasHeader = false

public var auto_disable_bodies: Bool
public var sor_pgs_precon_iters: UInt32
public var sor_pgs_iters: UInt32
public var sor_pgs_w: Float64
public var sor_pgs_rms_error_tol: Float64
public var contact_surface_layer: Float64
public var contact_max_correcting_vel: Float64
public var cfm: Float64
public var erp: Float64
public var max_contacts: UInt32

public init(auto_disable_bodies: Bool, sor_pgs_precon_iters: UInt32, sor_pgs_iters: UInt32, sor_pgs_w: Float64, sor_pgs_rms_error_tol: Float64, contact_surface_layer: Float64, contact_max_correcting_vel: Float64, cfm: Float64, erp: Float64, max_contacts: UInt32) {
self.auto_disable_bodies = auto_disable_bodies
self.sor_pgs_precon_iters = sor_pgs_precon_iters
self.sor_pgs_iters = sor_pgs_iters
self.sor_pgs_w = sor_pgs_w
self.sor_pgs_rms_error_tol = sor_pgs_rms_error_tol
self.contact_surface_layer = contact_surface_layer
self.contact_max_correcting_vel = contact_max_correcting_vel
self.cfm = cfm
self.erp = erp
self.max_contacts = max_contacts
}

public init() {
    auto_disable_bodies = Bool()
sor_pgs_precon_iters = UInt32()
sor_pgs_iters = UInt32()
sor_pgs_w = Float64()
sor_pgs_rms_error_tol = Float64()
contact_surface_layer = Float64()
contact_max_correcting_vel = Float64()
cfm = Float64()
erp = Float64()
max_contacts = UInt32()
}

}
}
