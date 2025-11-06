import tensorflow as tf
import numpy as np
from PIL import Image, ImageOps

# Load model and labels
model = tf.keras.models.load_model("tm_model.h5", compile=False)
class_names = open("tm_labels.txt", "r").readlines()

# Load a test image (replace with an actual file path)
image = Image.open("test.jpg").convert("RGB")

# Preprocess (resize to 224x224 like Teachable Machine expects)
size = (224, 224)
image = ImageOps.fit(image, size, Image.Resampling.LANCZOS)
image_array = np.asarray(image)

# Normalize to [0,1]
image_array = image_array.astype(np.float32) / 255.0
image_array = np.expand_dims(image_array, axis=0)

# Run inference
prediction = model.predict(image_array)
index = np.argmax(prediction)
confidence = prediction[0][index]

print("Predicted:", class_names[index].strip(), "with confidence:", confidence)
