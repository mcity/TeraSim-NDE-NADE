import redis
import json
import sys

def main():
    # Establish a connection to your Redis server
    r = redis.Redis(host='localhost', port=6379, db=0)
    # Create a new JSON object
    j = json.loads('{}')

    iteration = r.get('iteration')
    while iteration is None:
        iteration = r.get('iteration')
    
    hostname = "cosim_test_local"
    # hostname = r.get('hostname')
    # while hostname is None:
    #     hostname = r.get('hostname')
    #     print("No hostname found in Redis")

    iteration = iteration.decode("utf-8")
    print("Iteration number: " + iteration)
    folder_name = 'output/' + hostname + '/raw_data/cav_context_0_' + iteration

    av_context_str_prev = ""

    while True:
        launch_autoware = r.get('launch_autoware')
        if launch_autoware is not None:
            launch_autoware = launch_autoware.decode("utf-8")
            if launch_autoware == "0":
                print("Saving CAV context data to " + folder_name)
                with open(folder_name + '/cav_context.json', 'w') as f:
                    saved_data = json.dumps(j, indent=4)
                    saved_data = saved_data.replace("\\", "")
                    f.write(saved_data)
                    f.write("\n")
                print("Data saving complete, exiting")
                sys.exit(0)

        # Read data from Redis for the keys 'av_context' and 'time'
        av_context = r.get('av_context')
        time_value = r.get('terasim_time')

        if (av_context is None) or (time_value is None):
            continue

        # The data read from Redis is 'bytes' type, so decode to convert to string
        av_context_str = av_context.decode("utf-8")  
        time_str = time_value.decode("utf-8")

        if (av_context_str!= av_context_str_prev):
            # Assign the retrieved av_context to the time key in JSON object
            j[time_str] = av_context_str
            av_context_str_prev = av_context_str
            print("recording CAV context data")

if __name__ == "__main__":
    main()
