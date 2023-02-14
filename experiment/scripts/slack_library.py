import requests
import json
import yaml

def get_slack_webhook():
    with open('yaml/slack_webhook.yaml') as f:
        configs = yaml.load(f, Loader=yaml.FullLoader)
        slack_webhook = configs['webhook']
    return slack_webhook

def send_slack_message(payload, webhook):
    """Send a Slack message to a channel via a webhook. 
    
    Args:
        payload (dict): Dictionary containing Slack message, i.e. {"text": "This is a test"}
        webhook (str): Full Slack webhook URL for your chosen channel. 
    
    Returns:
        HTTP response code, i.e. <Response [503]>
    """

    return requests.post(webhook, json.dumps(payload))