// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:action/Cdcr.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__ACTION__DETAIL__CDCR__BUILDER_HPP_
#define INTERFACE__ACTION__DETAIL__CDCR__BUILDER_HPP_

#include "interface/action/detail/cdcr__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace interface
{

namespace action
{

namespace builder
{

class Init_Cdcr_Goal_goal_pose
{
public:
  Init_Cdcr_Goal_goal_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface::action::Cdcr_Goal goal_pose(::interface::action::Cdcr_Goal::_goal_pose_type arg)
  {
    msg_.goal_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::action::Cdcr_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::action::Cdcr_Goal>()
{
  return interface::action::builder::Init_Cdcr_Goal_goal_pose();
}

}  // namespace interface


namespace interface
{

namespace action
{

namespace builder
{

class Init_Cdcr_Result_result_pose
{
public:
  Init_Cdcr_Result_result_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface::action::Cdcr_Result result_pose(::interface::action::Cdcr_Result::_result_pose_type arg)
  {
    msg_.result_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::action::Cdcr_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::action::Cdcr_Result>()
{
  return interface::action::builder::Init_Cdcr_Result_result_pose();
}

}  // namespace interface


namespace interface
{

namespace action
{

namespace builder
{

class Init_Cdcr_Feedback_feedback_pose
{
public:
  Init_Cdcr_Feedback_feedback_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface::action::Cdcr_Feedback feedback_pose(::interface::action::Cdcr_Feedback::_feedback_pose_type arg)
  {
    msg_.feedback_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::action::Cdcr_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::action::Cdcr_Feedback>()
{
  return interface::action::builder::Init_Cdcr_Feedback_feedback_pose();
}

}  // namespace interface


namespace interface
{

namespace action
{

namespace builder
{

class Init_Cdcr_SendGoal_Request_goal
{
public:
  explicit Init_Cdcr_SendGoal_Request_goal(::interface::action::Cdcr_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::interface::action::Cdcr_SendGoal_Request goal(::interface::action::Cdcr_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::action::Cdcr_SendGoal_Request msg_;
};

class Init_Cdcr_SendGoal_Request_goal_id
{
public:
  Init_Cdcr_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cdcr_SendGoal_Request_goal goal_id(::interface::action::Cdcr_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Cdcr_SendGoal_Request_goal(msg_);
  }

private:
  ::interface::action::Cdcr_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::action::Cdcr_SendGoal_Request>()
{
  return interface::action::builder::Init_Cdcr_SendGoal_Request_goal_id();
}

}  // namespace interface


namespace interface
{

namespace action
{

namespace builder
{

class Init_Cdcr_SendGoal_Response_stamp
{
public:
  explicit Init_Cdcr_SendGoal_Response_stamp(::interface::action::Cdcr_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::interface::action::Cdcr_SendGoal_Response stamp(::interface::action::Cdcr_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::action::Cdcr_SendGoal_Response msg_;
};

class Init_Cdcr_SendGoal_Response_accepted
{
public:
  Init_Cdcr_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cdcr_SendGoal_Response_stamp accepted(::interface::action::Cdcr_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_Cdcr_SendGoal_Response_stamp(msg_);
  }

private:
  ::interface::action::Cdcr_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::action::Cdcr_SendGoal_Response>()
{
  return interface::action::builder::Init_Cdcr_SendGoal_Response_accepted();
}

}  // namespace interface


namespace interface
{

namespace action
{

namespace builder
{

class Init_Cdcr_GetResult_Request_goal_id
{
public:
  Init_Cdcr_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interface::action::Cdcr_GetResult_Request goal_id(::interface::action::Cdcr_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::action::Cdcr_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::action::Cdcr_GetResult_Request>()
{
  return interface::action::builder::Init_Cdcr_GetResult_Request_goal_id();
}

}  // namespace interface


namespace interface
{

namespace action
{

namespace builder
{

class Init_Cdcr_GetResult_Response_result
{
public:
  explicit Init_Cdcr_GetResult_Response_result(::interface::action::Cdcr_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::interface::action::Cdcr_GetResult_Response result(::interface::action::Cdcr_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::action::Cdcr_GetResult_Response msg_;
};

class Init_Cdcr_GetResult_Response_status
{
public:
  Init_Cdcr_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cdcr_GetResult_Response_result status(::interface::action::Cdcr_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_Cdcr_GetResult_Response_result(msg_);
  }

private:
  ::interface::action::Cdcr_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::action::Cdcr_GetResult_Response>()
{
  return interface::action::builder::Init_Cdcr_GetResult_Response_status();
}

}  // namespace interface


namespace interface
{

namespace action
{

namespace builder
{

class Init_Cdcr_FeedbackMessage_feedback
{
public:
  explicit Init_Cdcr_FeedbackMessage_feedback(::interface::action::Cdcr_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::interface::action::Cdcr_FeedbackMessage feedback(::interface::action::Cdcr_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::action::Cdcr_FeedbackMessage msg_;
};

class Init_Cdcr_FeedbackMessage_goal_id
{
public:
  Init_Cdcr_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cdcr_FeedbackMessage_feedback goal_id(::interface::action::Cdcr_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_Cdcr_FeedbackMessage_feedback(msg_);
  }

private:
  ::interface::action::Cdcr_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::action::Cdcr_FeedbackMessage>()
{
  return interface::action::builder::Init_Cdcr_FeedbackMessage_goal_id();
}

}  // namespace interface

#endif  // INTERFACE__ACTION__DETAIL__CDCR__BUILDER_HPP_
