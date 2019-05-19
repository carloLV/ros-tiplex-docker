import ros.*;
import com.fasterxml.jackson.databind.JsonNode;
public class Movment{
private RosBridge bridge;
	public Movment(RosBridge bridge){
	this.bridge=bridge;
	//insert your structure here
}

	/*****this is the basic code for publishers. You can modify it as you want*****/
	Publisher pub0 = new Publisher("/move_base_simple/goal", "geometry_msgs/PoseStamped", this.bridge);
	//Create your message object, uncomment this part and edit as you wish
	//pub.publish(new YourMessageType<T>(yourMessageContent));

	/*****this is the basic code for subscribers, including callback.
**You can modify it as you want*****/
	this.bridge.subscribe(SubscriptionRequestMsg.generate("/move_base/result")
			.setType("move_base_msgs/MoveBaseActionResult")
			.setThrottleRate(1)
			.setQueueLength(1),
		new RosListenDelegate() {
			@Override
			public void receive(JsonNode data, String repr){
				//Insert you message object and data here
				MessageUnpacker<YourMessageObj<Type>> unpacker = new MessageUnpacker<YourMessageObj<Type>>(YourMessageObj.class);
				YourMessageObj<Type> msg = unpacker.unpackRosMessage(data);
				//Your code to handle callback goes here. You can check messages an act basing on it
			}
		}
);


}