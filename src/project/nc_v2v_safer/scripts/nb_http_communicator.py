#!/usr/bin/env python3
import requests
import logging
import json
import uuid
import time
import redis
import os

# Configure logging
logging.basicConfig(level=logging.INFO)

class NbHttpCommunicator:
    def __init__(self, base_url, redis_host='localhost', redis_port=6379, redis_db=0):
        self.base_url = base_url
        self.session = requests.Session()
        self.connected = False
        self.auth_token = None
        self.redis_client = redis.StrictRedis(host=redis_host, port=redis_port, db=redis_db, password="navifra1@3$")

    def check_server_state(self):
        url = f"{self.base_url}/healthcheck"
        try:
            response = self.session.get(url, timeout=5)
            logging.info(f"Server response code: {response.status_code}")
            return response.status_code == 200 or response.status_code == 401
        except requests.RequestException as e:
            logging.error(f"Server check failed: {e}")
            return False

    def connect_server(self, host, port):
        logging.info(f"Connecting to {host}:{port}")
        self.base_url = f"http://{host}:{port}"
        if self.check_server_state():
            logging.info(f"{host}:{port} Connected")
            self.connected = True
            return True
        else:
            logging.error(f"{host}:{port} Connection Error")
            return False

    def get_robot_acs_reg_code(self):
        """
        Retrieves the robot_acs_reg_code from Redis.
        """
        try:
            # Redis에서 robot_acs_reg_code 값을 가져옵니다.
            reg_code = self.redis_client.get('robot_acs_reg_code')
            if reg_code:
                reg_code = reg_code.decode('utf-8')  # 바이트를 문자열로 변환
                logging.info(f"Retrieved robot_acs_reg_code: {reg_code}")
                return reg_code
            else:
                logging.error("robot_acs_reg_code not found in Redis.")
                return None
        except redis.RedisError as e:
            logging.error(f"Failed to get robot_acs_reg_code from Redis: {e}")
            return None

    def authenticate(self, account, password):
        url = f"{self.base_url}/auth/login"
        # logging.info(f"Sending POST request to {url} for authentication")
        
        auth_data = {
            "account": account,
            "password": password,
            "token": "string"
        }

        try:
            response = self.session.post(url, json=auth_data)

            if response.status_code == 200:
                result = response.json()
                if result.get('result') == 'success':
                    self.auth_token = result.get('token')
                    logging.info(f"Authentication successful. Token: {self.auth_token}")
                    return True
                else:
                    logging.error(f"Authentication failed: {result.get('message')}")
                    return False
            else:
                logging.error(f"Error: {response.status_code}, {response.text}")
                return False
        except requests.RequestException as e:
            logging.error(f"Authentication request failed: {e}")
            return False

    def get_request(self, endpoint):
        url = f"{self.base_url}/{endpoint}"
        # logging.info(f"Sending GET request to {url}")

        try:
            headers = {
                "Authorization": f"Bearer {self.auth_token}",
                "Content-Type": "application/json; charset=utf-8",
                "Connection": "keep-alive",
                "Keep-Alive": "timeout=5"
            }
            
            response = self.session.get(url, headers=headers)

            if response.status_code == 200:
                return response.json()
            else:
                logging.error(f"Error: {response.status_code}, {response.text}")
                return None
        except requests.RequestException as e:
            logging.error(f"GET request failed: {e}")
            return None

    def post_request(self, endpoint, data):
        url = f"{self.base_url}/{endpoint}"
        logging.info(f"Sending POST request to {url}")

        try:
            headers = {
                "Authorization": f"Bearer {self.auth_token}",
                "Content-Type": "application/json; charset=utf-8",
                "Connection": "keep-alive",
                "Keep-Alive": "timeout=5"
            }
            
            response = self.session.post(url, headers=headers, json=data)

            if response.status_code == 200:
                s_json = response.json()
                # logging.info(s_json)
                self.parse_response(s_json)
            else:
                logging.error(f"Error: {response.status_code}, {response.text}")
        except requests.RequestException as e:
            logging.error(f"POST request failed: {e}")
            logging.error(data)
    
    def parse_response(self, response):
        print(response)
    
    def update_active_robots(self):
        robots_data = self.get_request("robots")
        print(f"Robots data retrieved: {robots_data}")
        for robot in robots_data['list']:
            print(f"Processing robot {robot['id']}")
            robot_id = robot['id']

            # GET /robots/{robot_id}
            robot_details = self.get_request(f"robots/{robot_id}")
            if robot_details:
                print(f"Robot details retrieved for {robot_id}: {robot_details}")

                # POST /robots/{robot_id}/update
                update_data = {
                    # "model_type_cd": "D10001",
                    "is_enabled": True,
                    # "desc": "string",
                    # "alias_name": "string",
                    "job_is_active": True
                }
                self.post_request(f"robots/{robot_id}/update", update_data)   
                
    def update_inactive_robots(self):
        robots_data = self.get_request("robots")
        for robot in robots_data['list']:
            robot_id = robot['id']
            # GET /robots/{robot_id}
            robot_details = self.get_request(f"robots/{robot_id}")
            if robot_details:
                # POST /robots/{robot_id}/update
                update_data = {
                    # "model_type_cd": "D10001",
                    "is_enabled": False,
                    # "desc": "string",
                    # "alias_name": "string",
                    "job_is_active": True
                }
                self.post_request(f"robots/{robot_id}/update", update_data)   

    # num을 받아서 num만큼 로봇을 active 상태로 만들어줌
    def activate_robots(self, num):
        robots_data = self.get_request("robots")
        
        for robot in robots_data['list']:
            print(f"Processing robot {robot['id']}")
            robot_id = robot['id']

            # GET /robots/{robot_id}
            robot_details = self.get_request(f"robots/{robot_id}")
            if robot_details:
                print(f"Robot details retrieved for {robot_id}: {robot_details}")

                # POST /robots/{robot_id}/update
                update_data = {
                    # "model_type_cd": "D10001",
                    "is_enabled": True,
                    # "desc": "string",
                    # "alias_name": "string",
                    "job_is_active": True
                }
                self.post_request(f"robots/{robot_id}/update", update_data)   
                num -= 1
                if num == 0:
                    break     

    def register_robots(self, start, end):
        for i in range(start, end):
            # i가 1의 자리 수일 때, 00을 붙여줌
            if i < 10:
                i = f"00{i}"
            # i가 10의 자리 수일 때, 0을 붙여줌
            elif i < 100:
                i = f"0{i}"
                
            robot_data = {
                "model_type_cd": "D10003",
                "driving_type_cd": "D30001",
                "ip_addr": f"CORE{i}",
                "robot_acs_reg_code": self.get_robot_acs_reg_code()
            }
            self.post_request("robots/create", robot_data)
            print(f"Robot created: SALLY{i}")
        self.update_active_robots()
    
    def register_job(self):  
        main_map = self.get_request("scan-maps/main-info")
        main_map_id = main_map['item']['id']
        print(main_map_id)
        nodes = self.get_request(f"scan-maps/{main_map_id}/nodes")
        # print(nodes)
        cnt = 0
        for node in nodes["list"]:
            # if node['node_type_cd'] == 'D40003':
            node_id = node['id']
            node_name = node['name']
            # node_name에 Start가 포함되어 있지 않으면 job 생성
            if "Start" not in node_name:
                self.post_request("jobs/create", self.create_job_messge(main_map_id, node_id, "5ed2d97b-de7e-4c50-a29b-111111111019", node_name) )
                cnt += 1
                if cnt == 300:
                    break

                
    def create_job_messge(self, map_id, node_id, action_id, node_name):        
        job_data = {
            "id": str(uuid.uuid4()),
            "name": node_name,
            "acs_map_id": map_id,
            "priority": 1,
            "desc": "string",
            "is_enabled": True,
            "model_type_cd": "D10003",
            "tasks": [
                {
                "id": str(uuid.uuid4()),
                "job_id": str(uuid.uuid4()),
                "node_id": node_id,
                "action_id": action_id,
                "sort_index": 1
                }
            ],
            "allocates": [
                {
                "id": "",
                "job_id": "",
                "device_id": ""
                },
                {
                "id": "",
                "job_id": "",
                "device_id": ""
                }
            ]
        }
        return job_data

    def delete_jobs(self):
        jobs_data = self.get_request("jobs")
        if jobs_data and 'list' in jobs_data:
            for job in jobs_data['list']:
                job_id = job['id']
                self.post_request(f"jobs/{job_id}/delete", {})
                print(f"Job deleted: {job_id}")

    def delete_robots(self):
        robots_data = self.get_request("robots")
        if robots_data and 'list' in robots_data:
            for robot in robots_data['list']:
                robot_id = robot['id']
                self.post_request(f"robots/{robot_id}/delete", {})
                print(f"Robot deleted: {robot_id}")

    def find_amr(self, robot_ip):
        """
        Finds AMR (Autonomous Mobile Robot) information using the robot's IP address.
        """
        endpoint = f"robots/find-amr/{robot_ip}"
        response = self.get_request(endpoint)

        if response:
            robot_info = response.get('item')
            if robot_info:
                robot_id = robot_info.get('id')
                token = robot_info.get('token')
                is_enabled = robot_info.get('is_enabled')
                job_is_active = robot_info.get('job_is_active')

                # 로봇 정보 출력
                logging.info(f"Robot ID: {robot_id}, Token: {token}, Enabled: {is_enabled}, Active: {job_is_active}")
                
                return robot_id
            else:
                logging.error("No robot information found.")
                return None
        else:
            logging.error("Failed to find AMR.")
            return None
# Command Line Interface
def main():
    backend_host = os.getenv("BACKEND_HOST", "localhost")
    backend_port = os.getenv("BACKEND_PORT", "5000")
    redis_host = os.getenv("REDIS_HOST", "localhost")
    redis_port = int(os.getenv("REDIS_PORT", "6379"))

    # 서버 URL 구성
    base_url = f"http://{backend_host}:{backend_port}"
    communicator = NbHttpCommunicator(base_url=base_url, redis_host=redis_host, redis_port=redis_port)
    if communicator.connect_server(backend_host, backend_port):
        if communicator.authenticate("admin", "admin"):
            while True:
                print("Invalid choice. Please try again.")

if __name__ == "__main__":
    main()
