using phaseshift_interfaces.srv;
using ROS2;

namespace System.Service
{
    public class SetGoalClient : GenericServiceClient<SetGoal_Request, SetGoal_Response>
    {
        public SetGoalClient(ROS2Node node, string serviceName) : base(node, serviceName)
        {
        }
    }
}