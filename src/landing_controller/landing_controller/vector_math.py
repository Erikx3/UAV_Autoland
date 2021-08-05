from geometry_msgs.msg import Vector3, Transform, Quaternion



def vec_add(v1: Vector3, v2: Vector3):
    v = Vector3()

    v.x = v1.x + v2.x
    v.y = v1.y + v2.y
    v.z = v1.z + v2.z

    return v

def quat_multiply(q1: Quaternion, q2: Quaternion):
    q = Quaternion()
    q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z 
    q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y 
    q.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x 
    q.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w

    return q

def quat_inv(q: Quaternion):
    q_ret = Quaternion()

    q_ret.w = q.w
    q_ret.x = -q.x
    q_ret.y = -q.y
    q_ret.z = -q.z

    return q_ret

def quat2vec(q: Quaternion):
    v = Vector3()

    v.x = q.x
    v.y = q.y
    v.z = q.z

    return v

def vec2quat(v: Vector3):
    q = Quaternion()

    q.w = 0.
    q.x = v.x
    q.y = v.y
    q.z = v.z

    return q

def quat_rotate_vec(v: Vector3, q_rot:Quaternion):
    return quat2vec( quat_multiply(quat_multiply(q_rot, vec2quat(v)), quat_inv(q_rot) ) )


def transform_apply(v: Vector3, tf: Transform):
    """ Transform object by the tf transform tf

    :param object: object to be transform
    :param tf: Transform Transformation that shall be applied
    :return: transformed object
    """

    return vec_add(quat_rotate_vec(v, tf.rotation), tf.translation)