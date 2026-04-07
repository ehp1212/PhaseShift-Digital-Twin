using phaseshift_interfaces.srv;
using ROS2;

namespace System.Service
{
    public class GetVoxelMapClient : GenericServiceClient<GetVoxelMapPath_Request, GetVoxelMapPath_Response>
    {
        public GetVoxelMapClient(ROS2Node node, string serviceName) : base(node, serviceName)
        {
        }
    }
}