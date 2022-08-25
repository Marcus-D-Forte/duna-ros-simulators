#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  // This plugin simply moves the model associated with it.
  class ModelPush : public ModelPlugin
  {
  private:
    double initial_position_x;
    double threshold = 5;
    

  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      std::cout << "Model loaded!\n";
      initial_position_x = this->model->WorldPose().X();

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
  public:
    void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0))
      static ignition::math::Pose3d pose = this->model->WorldPose();

      double x = pose.X() + 0.0005;

      if (x > initial_position_x + threshold)
      {
        pose.SetX(initial_position_x);
      }
      else
      {
        pose.SetX(x);
      }

      this->model->SetWorldPose(pose);
    }

    // Pointer to the model
  private:
    physics::ModelPtr model;

    // Pointer to the update event connection
  private:
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}