# ntrip_ros
NTRIP client, imports RTCM streams to ROS topic

This was forked from https://github.com/ros-agriculture/ntrip_ros

The CORS correction server that I am using does not have the /n/r characters. So I parsed out individual messages and published each one on the /rtcm ROS topic. It would crash with IncompleteRead error. I added patch at top of file. But the connection had closed and it would crash again. I ended up detecting zero length data and closing and reopening the data stream. It continues on without a glitch.

You can generate the require $GPGGA message at this site. https://www.nmeagen.org/ Set a point near where you want to run and click "Generate NMEA file". Cut and paste the $GPGGA message into the launch file. 

The preferable server for NTRIP is [http://www.euref-ip.net](http://www.euref-ip.net)  
You can register for credentials on the server [here](https://register.rtcm-ntrip.org/cgi-bin/registration.cgi)  
All streams are listed [here](https://igs.bkg.bund.de/root_ftp/NTRIP/streams/streamlist_euref-ip.htm). Prefereble streams when working in CERTH are: *AUT100GRC0*, *PAT000GRC0*, *DYNG00GRC0*  

Streams are not always online, [this page](epncb.oma.be/_networkdata/data_access/real_time/status.php) will let you know if the stream is offline and causes the connection to fail.  

Tested with Ublox ZEDF9P  