file_setup=known.targets

while read -r symb mac ip role source destination; do
    [[ "$symb" == '#' ]] && continue
    [[ "$role" == 'server' ]] && mac_server=$mac 
    [[ "$role" == 'client' ]] && mac_client=$mac
    [[ "$role" == 'both' ]] && mac_server=$mac && mac_client=$mac        
done <$file_setup

echo 'ROS Setup to Server ('$mac_server') and Client ('$mac_client')'

declare -A found_mac2ip
found_server=0
found_client=0

while ! [[ "$found_server $found_client" == '1 1' ]]; do
    echo 'Trying to find the devices mapped in '${file_setup}$'\n'
    found_server=0
    found_client=0

    nmap_output=$(sudo nmap -n -sP --max-parallelism 100 192.168.1.0/24)

    echo 'Devices found:'
    while IFS=$' \t\n()' read -r f1 f2 f3 f4 f5 _; do
        [[ "$f1 $f2 $f3 $f4" == 'Nmap scan report for' ]] && ip=$f5

        if [[ "$f1 $f2" == 'MAC Address:' ]]; then
            mac_adress=$f3
            found_mac2ip[$mac_adress]=$ip && echo 'MAC: '$mac_adress ' IP: '${found_mac2ip[$mac_adress]}
        elif [[ "$f1 $f2 $f3" == 'Host is up.' ]]; then
            mac_adress=$(cat /sys/class/net/$(ip route show default | awk '/default/ {print $5}')/address)
            mac_adress=${mac_adress^^}
            found_mac2ip[$mac_adress]=$ip && echo 'MAC: '$mac_adress ' IP: '${found_mac2ip[$mac_adress]} '(this device)'
        fi

        [[ "$mac_adress" == "$mac_server" ]] && found_server=1
        [[ "$mac_adress" == "$mac_client" ]] && found_client=1
    done <<<"$nmap_output"

    echo $'\n'
done

while read -r symb mac ip role source destination; do
    if [[ "$symb" == '#' ]]; then
        continue
    fi

    ip=${found_mac2ip[$mac]-$ip}
    if [[ "$role" == 'both' ]]; then
        sed -i "s/^$ ${mac}.*/$ ${mac} ${ip} both/" ${file_setup}
        ROS_MASTER_URI='http://'${ip}':11311'
        ROS_IP=${ip}':11311'
    elif [[ "$role" == 'server' ]]; then
        sed -i "s/^$ ${mac}.*/$ ${mac} ${ip} server/" ${file_setup}
        ROS_MASTER_URI='http://'${ip}':11311'
    elif [[ "$role" == 'client' ]]; then
        sed -i "s/^$ ${mac}.*/$ ${mac} ${ip} client/" ${file_setup}
        ROS_IP=${ip}':11311'
    fi
done <$file_setup

echo 'ROS_MASTER_URI = '${ROS_MASTER_URI}
echo 'ROS_IP = '${ROS_IP}