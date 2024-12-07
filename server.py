import base64
from openai import OpenAI
from flask import Flask, request
import os
from rich.console import Console
from rich.bar import Bar

app = Flask(__name__)

client = OpenAI(api_key = os.getenv("OPENAI_API_KEY"))

recyclable = "false"
recycle_list = []

UPLOAD_FOLDER = "./uploads"
app.config["UPLOAD_FOLDER"] = UPLOAD_FOLDER
app.config['MAX_CONTENT_LENGTH'] = 16 * 1024 * 1024  # Limit file size to 16 MB

if not os.path.exists(UPLOAD_FOLDER):
    os.makedirs(UPLOAD_FOLDER)

def plot_bar_graph():
    true_count = recycle_list.count(True)
    false_count = recycle_list.count(False)

    console = Console()
    console.print("Recyclable :", Bar(size=true_count, begin=0, end=10, color="green"))
    console.print("Not Recyclable :", Bar(size=false_count, begin=0, end=10, color="red"))

@app.route('/recyclable', methods=['GET'])
def is_recyclable():
    print("is_recyclable: " + recyclable)
    return recyclable 

def set_recyclable(response):
    return "true" if response.upper() == "YES" else "false"

@app.route('/upload', methods=['POST'])
def upload_file():
    if 'file' not in request.files:
        return "No file part", 400
    file = request.files['file']
    if file.filename == '':
        return "No selected file", 400

    file_path = os.path.join(app.config["UPLOAD_FOLDER"], file.filename)
    file.save(file_path)

    # Process image with OpenAI (if needed)
    response = process_with_openai(file_path)

    print(response)

    # Set global variable for recyclable for GET requests
    global recyclable
    recyclable = set_recyclable(response)
    
    global recycle_list
    r = True if recyclable == "true" else False
    recycle_list.append(r)
    
    plot_bar_graph()

    # Return OpenAI response
    return response, 200


# Function to encode the image
def encode_image(image_path):
  with open(image_path, "rb") as image_file:
    return base64.b64encode(image_file.read()).decode('utf-8')


def process_with_openai(image_path):
    prompt = "Please identify if this object is recyclable or not. Only reply \"YES\" if it is recyclable or \"NO\" if it is not."

    base64_image = encode_image(image_path)

    response = client.chat.completions.create(
        model="gpt-4o",
        messages=[
          {
            "role": "user",
            "content": [
              {
                "type": "text",
                "text": prompt,
              },
              {
                "type": "image_url",
                "image_url": {
                  "url":  f"data:image/jpeg;base64,{base64_image}"
                },
              },
            ],
          }
        ],
    )
    return response.choices[0].message.content


if __name__ == "__main__":
    app.run(debug=True, host='0.0.0.0', port=5000)
