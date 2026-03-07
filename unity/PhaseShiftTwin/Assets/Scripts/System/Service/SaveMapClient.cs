using phaseshift_interfaces.srv;
using ROS2;

namespace System.Service
{
    public class SaveMapClient : GenericServiceClient<SaveMap_Request, SaveMap_Response>
    {
        public SaveMapClient(ROS2Node node, string serviceName) : base(node, serviceName)
        {
        }
    }
}