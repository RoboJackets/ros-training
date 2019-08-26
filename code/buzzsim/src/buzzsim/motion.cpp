#include <cmath>

#include <buzzsim/motion.h>
#include <iostream>
#include <iomanip>
#include <tf/transform_datatypes.h>

namespace motion
{
Twist TwistLimits::limit(const Twist& twist) const
{
  double new_linear = std::clamp(twist.linear, -linear, linear);
  double new_angular = std::clamp(twist.angular, -angular, angular);

  return { new_linear, new_angular };
}
bool TwistLimits::operator==(const TwistLimits &other) const
{
  return linear == other.linear && angular == other.angular;
}

void Twist::apply(const Acceleration& acceleration, const TwistLimits& limits)
{
  linear += acceleration.linear * SECS_PER_UPDATE;
  angular += acceleration.angular * SECS_PER_UPDATE;

  limits.limit(*this);
}

Twist Twist::getApplied(const Acceleration& acceleration, const TwistLimits& limits) const
{
  double new_linear = linear + acceleration.linear * SECS_PER_UPDATE;
  double new_angular = angular + acceleration.angular * SECS_PER_UPDATE;

  return limits.limit({ new_linear, new_angular });
}
Twist Twist::operator+(const Twist& other) const
{
  return { linear + other.linear, angular + other.angular };
}

Twist Twist::operator/(double scalar) const
{
  return { linear / scalar, angular / scalar };
}

bool Twist::operator==(const Twist& other) const
{
  return linear == other.linear && angular == other.angular;
}

void Pose::apply(const Twist& twist)
{
  position.x += cos(orientation) * twist.linear * SECS_PER_UPDATE;
  position.y += sin(orientation) * twist.linear * SECS_PER_UPDATE;
  orientation += twist.angular * SECS_PER_UPDATE;

  normalizeHeading();
}

Pose Pose::getApplied(const Twist& twist) const
{
  double new_x = position.x + cos(orientation) * twist.linear * SECS_PER_UPDATE;
  double new_y = position.y + sin(orientation) * twist.linear * SECS_PER_UPDATE;
  double new_orientation = orientation + twist.angular * SECS_PER_UPDATE;

  Pose new_pose{ { new_x, new_y }, new_orientation };
  new_pose.normalizeHeading();

  return new_pose;
}

void Pose::normalizeHeading()
{
  orientation = -M_PI + fmod(2 * M_PI + fmod(orientation + M_PI, 2 * M_PI), 2 * M_PI);
}

bool Pose::operator==(const Pose& other) const
{
  return position == other.position && orientation == other.orientation;
}

geometry_msgs::Pose Pose::toROSMsg() const
{
  geometry_msgs::Pose msg{};
  msg.position.x = position.x;
  msg.position.y = position.y;
  msg.orientation = tf::createQuaternionMsgFromYaw(orientation);

  return msg;
}

State::State(Pose pose, Twist twist) : pose{ pose }, twist{ twist }
{
}

void State::apply(const Acceleration& acceleration, const Limits& limits)
{
  auto limited_acceleration = limits.acceleration.limit(acceleration);
  Twist new_twist = twist.getApplied(limited_acceleration);
  Twist limited_new_twist = limits.twist.limit(new_twist);

  // Not saturating, use midpoint
  if (new_twist == limited_new_twist)
  {
    Twist midpoint = (twist + new_twist) / 2;
    pose.apply(midpoint);
  }
  else
  {
    pose.apply(limited_new_twist);
  }

  twist = limited_new_twist;
}

bool State::operator==(const State &other) const
{
  return pose == other.pose && twist == other.twist;
}

Acceleration AccelerationLimits::limit(const Acceleration& acceleration) const
{
  double new_linear = std::clamp(acceleration.linear, -linear, linear);
  double new_angular = std::clamp(acceleration.angular, -angular, angular);
  return { new_linear, new_angular };
}
bool AccelerationLimits::operator==(const AccelerationLimits &other) const
{
  return linear == other.linear && angular == other.angular;
}

bool Acceleration::operator==(const Acceleration& other) const
{
  return linear == other.linear && angular == other.angular;
}

QPointF Position::toQPointF(int width, int height) const
{
  // x is front, y is left.
  return { 0.5 * width - PIXELS_PER_M * y, 0.5 * height - PIXELS_PER_M * x };
}

bool Position::operator==(const Position &other) const
{
  return x == other.x && y == other.y;
}

std::ostream& operator<<(std::ostream& out, const Position& position)
{
  out << std::fixed << std::setprecision(3);
  out << "Position(" << position.x << ", " << position.y << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out, const Pose& pose)
{
  out << std::fixed << std::setprecision(3);
  out << "Pose(" << pose.position << ", " << pose.orientation << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out, const Twist& twist)
{
  out << std::fixed << std::setprecision(3);
  out << "Twist(" << twist.linear << ", " << twist.angular << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out, const Acceleration& acceleration)
{
  out << std::fixed << std::setprecision(3);
  out << "Acceleration(" << acceleration.linear << ", " << acceleration.angular << ")";
  return out;
}

std::ostream& operator<<(std::ostream& out, const State& state)
{
  out << std::fixed << std::setprecision(3);
  out << "State(" << state.pose << ", " << state.twist << ")";
  return out;
}

bool Limits::operator==(const Limits &other) const
{
  return acceleration == other.acceleration && twist == other.twist;
}
}  // namespace motion
