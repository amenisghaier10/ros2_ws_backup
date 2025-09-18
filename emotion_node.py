from transformers import pipeline
import rclpy 
from rclpy.node import Node 
from std_msgs.msg import String

class EmotionDetectorNode(Node):
    def __init__(self):
        super().__init__('emotion_node')
        self.publisher_ = self.create_publisher(String, 'emotion' , 10)
        self.subscription = self.create_subscription( String,  'input_text', self.detect_emotion, 10)
        self.classifier = pipeline("text-classification" , model="j-hartmann/emotion-english-distilroberta-base" , return_all_scores=True)   
        self.get_logger().info("EmotionDetectorNode has started and is listening for input...")

    def detect_emotion(self, msg):
        text = msg.data
        result = self.classifier(text)
        top_emotion = max(result[0], key=lambda x: x['score']) 
        emotion_msg = String()
        emotion_msg.data = f"Detected emotion: {top_emotion['label']} ({top_emotion['score']:.2f})"
        self.publisher_.publish(emotion_msg)
        self.get_logger().info(emotion_msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = EmotionDetectorNode()
    try:
       rclpy.spin(node)
    except KeyboardInterrupt:
       pass
    rclpy.shutdown()

if __name__ == '__main__' :
    main()
