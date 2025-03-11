import requests
import json


def main():
        
    url = "https://qianfan.baidubce.com/v2/app/conversation"
    
    payload = json.dumps({
        "app_id": "fd963b90-de1c-4926-8575-a86af990fba6"
    }, ensure_ascii=False)
    headers = {
        'Content-Type': 'application/json',
        'X-Appbuilder-Authorization': 'Bearer bce-v3/ALTAK-Ph5HKAos7Eie98jFnAQdx/66935b2d5d92626966e9d0b5e81c9ddaf0439ef9'
    }
    
    response = requests.request("POST", url, headers=headers, data=payload.encode("utf-8"))
    
    print(response.text)
    

if __name__ == '__main__':
    main()

