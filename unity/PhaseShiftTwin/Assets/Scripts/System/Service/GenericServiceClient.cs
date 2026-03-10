using System.Threading.Tasks;
using ROS2;
using UnityEngine;

namespace System.Service
{
    public class GenericServiceClient<TReq, TRes>
        where TReq : Message, new()
        where TRes : Message, new()
    {
        IClient<TReq, TRes> _client;
        
        public GenericServiceClient(ROS2Node node, string serviceName)
        {
            _client = node.CreateClient<TReq, TRes>(serviceName);
        }

        public async Task<TRes> CallAsync(TReq request)
        {
            while (!_client.IsServiceAvailable())
            {
                await Task.Delay(200);
            }
         
            return await _client.CallAsync(request);
        }
    }
}
