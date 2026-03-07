using phaseshift_interfaces.srv;
using ROS2;

namespace System.Service
{
    public class NavigateClient : GenericServiceClient<NavigateToGoal_Request, NavigateToGoal_Response>
    {
        public NavigateClient(ROS2Node node, string serviceName) : base(node, serviceName)
        {
        }
    }
}