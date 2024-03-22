#ifndef DYNAMIC_PUBLISHER_H
#define DYNAMIC_PUBLISHER_H

#include <mrs_lib/publisher_handler.h>

namespace task_01_wrapper
{

#include <ros/ros.h>

template <class T>
class DynamicPublisher {

public:
  DynamicPublisher(){};
  DynamicPublisher(ros::NodeHandle& nh);
  DynamicPublisher(const DynamicPublisher<T>& other);
  DynamicPublisher<T>& operator=(const DynamicPublisher<T>& other);

  void publish(const std::string name, const T& value);

private:
  ros::NodeHandle nh_;

  std::map<std::string, mrs_lib::PublisherHandler<T>> publishers_;
};

template <class T>
DynamicPublisher<T>::DynamicPublisher(ros::NodeHandle& nh) {

  nh_ = nh;
}

template <class T>
DynamicPublisher<T>::DynamicPublisher(const DynamicPublisher<T>& other) {

  nh_ = other.nh_;
}

template <class T>
DynamicPublisher<T>& DynamicPublisher<T>::operator=(const DynamicPublisher<T>& other) {

  if (this == &other) {
    return *this;
  }

  nh_ = other.nh_;

  return *this;
}

template <class T>
void DynamicPublisher<T>::publish(const std::string name, const T& value) {

  typename std::map<std::string, mrs_lib::PublisherHandler<T>>::iterator it;

  it = publishers_.find(name);

  if (it == publishers_.end()) {

    publishers_.insert(std::pair<std::string, mrs_lib::PublisherHandler<T>>(name, mrs_lib::PublisherHandler<T>(nh_, name)));
    it = publishers_.find(name);
  }

  it->second.publish(value);
}

}  // namespace task_01_wrapper

#endif  // DYNAMIC_PUBLISHER_H
