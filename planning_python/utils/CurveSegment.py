from math import sqrt, sin, cos, pi, floor, radians, copysign

class CurveSegment:
  @staticmethod
  def end_pose(start_pose, curvature, length):
    """Returns end pose, given start pose, curvature and length."""
    x, y, theta = start_pose
    if curvature == 0.0:
      # Linear movement.
      x += length * cos(theta)
      y += length * sin(theta)
      return (x, y, theta)
    else:
      # Curve segment of radius 1/curvature.
      tx = cos(theta)
      ty = sin(theta)
      radius = 1.0/curvature
      xc = x - radius * ty  # Center of circle.
      yc = y + radius * tx
      angle = length / radius
      cosa = cos(angle)
      sina = sin(angle)
      nx = xc + radius * (cosa * ty + sina * tx)
      ny = yc + radius * (sina * ty - cosa * tx)
      ntheta = (theta + angle + pi) % (2*pi) - pi
      return (nx, ny, ntheta)

  @staticmethod
  def segment_points(start_pose, curvature, length, delta_length):
    """Return points of segment, at delta_length intervals."""
    l = 0.0
    delta_length = copysign(delta_length, length)
    points = []
    while abs(l) < abs(length):
        points.append(CurveSegment.end_pose(start_pose, curvature, l))
        l += delta_length
    return points