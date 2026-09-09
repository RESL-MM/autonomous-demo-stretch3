class CobraAPIClient:
    def __init__(self,raw_data_dir,dir_date):
        self.logger = setup_logger("FetchingAPIData")
        #API Cred+Setting
        self.API_IP_Port = os.getenv('Cobra_API_IP_Port')
        self.API_username:str = os.getenv('Cobra_API_USERNAME')
        self.API_password:str = os.getenv('Cobra_API_PASSWORD')
        self.API_URL:str = f'http://{self.API_IP_Port}/api/v1.0' 
        self.dir_date = dir_date
        self.Data_time = CLI_DateMaster.GetTimestamp(get_data_date=self.dir_date,call_source="API",log_obj=self.logger)
        self.dir_filename_date = self.Data_time[2]
        self.raw_data_dir=raw_data_dir
        self.raw_data_dir_dated = f'{self.raw_data_dir}{self.dir_filename_date}/'
        
    def __enter__(self):
        #API Test
        if not self.API_IP_Port:
            self.logger.critical("Please set environment variables for Cobra API connection (Cobra_API_IP_Port) For Eg: 10.1.1.100:7389")
            raise ValueError("Please set environment variables for Cobra API connection (Cobra_API_IP_Port) For Eg: 10.1.1.100:7389")
        self.PingRESTAPI(self.API_IP_Port)
        #Creating The Required Folder
        self.EnvironmentSetup(self.raw_data_dir, self.raw_data_dir_dated)
        self.DataFetching_result=self.DataFetching()
    def __exit__(self):
        for handler in list(self.logger.handlers):
            self.logger.removeHandler(handler)
            
    def APIConnection(self,url):
        if not all([self.API_username, self.API_password]):
            self.logger.critical("Please set environment variables for Cobra API connection (Cobra_API_USERNAME, Cobra_API_PASSWORD)")
            raise ValueError("Please set environment variables for Cobra API connection (Cobra_API_USERNAME, Cobra_API_PASSWORD)")
        
        try:
            self.logger.debug(f"Requesting:{url}")
            response = requests.get(url, auth=(self.API_username, self.API_password))
            response.raise_for_status()  # This will raise an exception for HTTP error codes
            return response
        except requests.exceptions.HTTPError as e:
            self.logger.critical(f"HTTPError: {e}")
            raise
        except requests.exceptions.RequestException as e:
            self.logger.critical(f"ConnectionError: {e}")
            raise

    def DataFetching(self):
        self.GetCobraAuditLogs(self.Data_time[0], self.Data_time[1])
        self.GetCobraConfig()
        #Fetching and Saving the Raw-Data from API into CSV files
        
        if self.GetCobraMetaData(start=self.Data_time[0], end=self.Data_time[1]):
            if self.GetCobraSensorData(start=self.Data_time[0], end=self.Data_time[1]):
                self.logger.info(f"Data Download Successful! For the Date {self.dir_date.strftime('%Y-%m-%d')}")
                return True
            else:
                return(f"No Sensor data found between {self.Data_time[0]} & {self.Data_time[1]}.")    
                # raise ConnectionError(f"Error while getting the sensor data for the date {self.dir_date.strftime('%Y-%m-%d')}.")
        else:
            return(f"No Meta data found between {self.Data_time[0]} & {self.Data_time[1]}.")
        
    def PingRESTAPI(self,api_url):
        try:
            api_url=api_url.split(':')
            # Create a socket object
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            # Attempt to connect to the host and port
            sock.settimeout(5)
            sock.connect((api_url[0],int(api_url[1])))
        except (socket.timeout, ConnectionRefusedError, OSError):
            # If connection fails, return False
            self.logger.error(f"Cobra API is not Reachable on IP: {api_url}")
        finally:
            # Close the socket
            sock.close()
    
    def CreateRequiredDir(self,path):
        if not os.path.exists(path):
            os.makedirs(path)
            self.logger.info(f"Folder '{path}' created successfully.")
      
    def EnvironmentSetup(self,raw_data_dir, raw_data_dir_dated):
        if not os.path.exists(raw_data_dir_dated):
            self.CreateRequiredDir(raw_data_dir)
        if os.path.exists(raw_data_dir_dated):
            try:
                shutil.rmtree(raw_data_dir_dated)
                self.logger.warning(f"Directory '{raw_data_dir_dated}' deleted successfully.")
                self.CreateRequiredDir(raw_data_dir_dated)
            except OSError as e:
                self.logger.critical(f"Error: {raw_data_dir_dated} : {e.strerror}")
        else:
            self.CreateRequiredDir(raw_data_dir_dated)

    def GetCobraAuditLogs(self,startTime, endTime):
        url = f'{self.API_URL}/AuditLog/&From={startTime}&To={endTime}'
        response = self.APIConnection(url)
        if response.status_code == 200:
            config_file_name = f'{self.raw_data_dir_dated}audit_logs_{self.dir_filename_date}.json'
            with open(config_file_name, 'wb') as f:
                f.write(response.content)
            self.logger.info(f"Saved: {config_file_name}")
            return True
        else:
            self.logger.error("Failed to fetch config. Status code:", response.status_code)

    def GetCobraConfig(self):
        url = f'{self.API_URL}/Config'
        response = self.APIConnection(url)
        if response.status_code == 200:
            config_file_name = f'{self.raw_data_dir_dated}config_{self.dir_filename_date}.xml'
            with open(config_file_name, 'wb') as f:
                f.write(response.content)
            self.logger.info(f"Saved: {config_file_name}")
            return True
        else:
            self.logger.error("Failed to fetch config. Status code:", response.status_code)

    def GetCobraMetaData(self,start, end):
        url = f'{self.API_URL}/Jobs/'
        url = f'{url}&From={start}&To={end}'
        response = self.APIConnection(url)
        if response.status_code == 200:
            meta_file_name = f'{self.raw_data_dir_dated}meta_data_{self.dir_filename_date}.json'
            with open(meta_file_name, 'wb') as f:
                f.write(response.content)
            self.logger.info(f"Saved: {meta_file_name}")
            return True
        else:
            self.logger.error("Failed to fetch config. Status code:", response.status_code)

    def GetCobraSensorData(self,start, end):
        base_url = f'{self.API_URL}/Data/Module/'
        module_name = 'PMC1'
        start_time = start
        end_time = end
        format_type = '2'
        interval_ms = '250'
        add_header = 'true'
        add_alarms = 'true'
        add_recipe_starts = 'true'
        add_step_starts = 'true'
        add_phase_starts = 'true'
        url = f'{base_url}&Name={module_name}&Start={start_time}&End={end_time}&Format={format_type}?intervalMs={interval_ms}&addHeader={add_header}&addAlarms={add_alarms}&addRecipeStarts={add_recipe_starts}&addStepStarts={add_step_starts}&addPhaseStarts={add_phase_starts}'
        response = self.APIConnection(url)
        if response.status_code == 200:
            self.logger.info(f"UID: {response.text}")
            return self.GetDownloadDataStatus(response.text)
        else:
            self.logger.error(f"Error: {response.status_code}")

    def GetDownloadDataStatus(self,uid, processing=None):
        if processing is None:processing=0
        url = f'{self.API_URL}/Data/Status/'
        url = f"{url}&id={uid[1:-1]}"
        response = self.APIConnection(url)
        if response.status_code == 200:
            while int(response.text) != 3:
                if int(response.text) == 1:
                    if processing == 0:self.logger.info(f'Please Wait Data is Processing for date:{self.dir_filename_date}...')
                    else:self.logger.info(f"Still Processing for date:{self.dir_filename_date}...")
                    sleep(10)
                    return self.GetDownloadDataStatus(uid, processing=1)
                elif int(response.text) == 2:
                    return(f"Error: No Data Please check the Start and End Date for the folder {self.dir_filename_date}")
            return self.DownloadSensorData(uid=uid)

    def DownloadSensorData(self,uid):
        url = f'{self.API_URL}/Data/Download/'
        url = f"{url}&id={uid[1:-1]}"
        response = self.APIConnection(url)
        if response.status_code == 200:
            output_file = f'{self.raw_data_dir_dated}sensor_data_{self.dir_filename_date}_{self.Data_time[1][:-18]}.zip'
            with open(output_file, 'wb') as f:
                f.write(response.content)
            self.logger.info(f"Saved: {output_file}")
            return self.UnzipAndRename(output_file)
        else:
            return(f"Failed to download Sensor Data file. Status code: {response.status_code}")

    def UnzipAndRename(self,path):
        with zipfile.ZipFile(path, 'r') as zip_ref:
            zip_ref.extractall(self.raw_data_dir_dated)
        extracted_files = os.listdir(self.raw_data_dir_dated)
        for file_name in extracted_files:
            file_name_split = os.path.splitext(file_name)
            if file_name_split[1] == '.xml':
                if self.dir_filename_date in file_name:
                    pass
                else:
                    try:
                        new_file_name = f"{file_name_split[0]}_{self.dir_filename_date}{file_name_split[1]}"
                        os.rename(f"{self.raw_data_dir_dated}{file_name}", f"{self.raw_data_dir_dated}{new_file_name}")
                        self.logger.info(f"Renamed {file_name} to {new_file_name}")
                    except FileExistsError:
                        self.logger.warning(f"{new_file_name} already exists.")
            elif file_name_split[0].startswith("PMC1 "):
                try:
                    new_file_name = f"raw_sensor_data_{self.dir_filename_date}.csv"
                    os.rename(f"{self.raw_data_dir_dated}{file_name}", f"{self.raw_data_dir_dated}{new_file_name}")
                    self.logger.info(f"Renamed {file_name} to {new_file_name}")
                except FileExistsError:
                    self.logger.warning(f"{new_file_name} already exists.")
        return True