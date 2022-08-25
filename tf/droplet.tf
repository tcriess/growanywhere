resource "digitalocean_project" "growanywhere" {
  name        = "Growanywhere"
  description = "Growanywhere - data collection and homepage growanywhere.de monitor.growanywhere.de"
  purpose     = "IoT"
  // environment = "Development"
  resources = [
    digitalocean_droplet.growanywhere.urn,
    digitalocean_reserved_ip.growanywhere.urn,
    digitalocean_volume.growanywhere.urn
  ]
}

resource "digitalocean_reserved_ip" "growanywhere" {
  region = var.region
  lifecycle {
    prevent_destroy = true
  }
}

resource "digitalocean_reserved_ip_assignment" "growanywhere" {
  ip_address = digitalocean_reserved_ip.growanywhere.ip_address
  droplet_id = digitalocean_droplet.growanywhere.id
}

resource "digitalocean_volume" "growanywhere" {
  region                  = var.region
  name                    = "growanywhere"
  size                    = var.volume-size
  initial_filesystem_type = "ext4"
  lifecycle {
    prevent_destroy = true
  }
}

resource "digitalocean_volume_attachment" "growanywhere" {
  droplet_id = digitalocean_droplet.growanywhere.id
  volume_id  = digitalocean_volume.growanywhere.id
}

resource "digitalocean_droplet" "growanywhere" {
  name      = "growanywhere"
  image     = "ubuntu-22-04-x64"
  size      = var.size
  region    = var.region
  ipv6      = true
  user_data = <<EOF
#cloud-config
hostname: growanywhere
manage_etc_hosts: true
mounts:
  - [ "/dev/disk/by-id/scsi-0DO_Volume_${digitalocean_volume.growanywhere.name}", "/vol", "auto", "defaults,nofail", "0", "0" ]
package_upgrade: true
apt:
  sources:
    influxdata.list:
      source: "deb https://repos.influxdata.com/ubuntu jammy stable"
      key: |
        -----BEGIN PGP PUBLIC KEY BLOCK-----                                                                                                                         
        Version: GnuPG v1                                                                                                                                            
                                                                                                                                                                    
        mQINBFYJmwQBEADCw7mob8Vzk+DmkYyiv0dTU/xgoSlp4SQwrTzat8MB8jxmx60l                                                                                             
        QjmhqEyuB8ho4zzZF9KV+gJWrG6Rj4t69JMTJWM7jFz+0B1PC7kJfNM+VcBmkTnj                                                                                             
        fP+KJjqz50ETnsF0kQTG++UJeRYjG1dDK0JQNQJAM6NQpIWJI339lcDf15vzrMnb                                                                                             
        OgIlNxV6j1ZZqkle4fvScF1NQxYScRiL+sRgVx92SI4SyD/xZnVGD/szB+4OCzah                                                                                             
        +0Q/MnNGV6TtN0RiCDZjIUYiHoeT9iQXEONKf7T62T4zUafO734HyqGvht93MLVU                                                                                             
        GQAeuyx0ikGsULfOsJfBmb3XJS9u+16v7oPFt5WIbeyyNuhUu0ocK/PKt5sPYR4u                                                                                             
        ouPq6Ls3RY3BGCH9DpokcYsdalo51NMrMdnYwdkeq9MEpsEKrKIN5ke7fk4weamJ                                                                                             
        BiLI/bTcfM7Fy5r4ghdI9Ksw/ULXLm4GNabkIOSfT7UjTzcBDOvWfKRBLX4qvsx4                                                                                             
        YzA5kR+nX85u6I7W10aSqBiaLqk6vCj0QmBmCjlSeYqNQqSzH/6OoL6FZ7lP6AiG                                                                                             
        F2NyGveJKjugoXlreLEhOYp20F81PNwlRBCAlMC2Q9mpcFu0dtAriVoG4gVDdYn5                                                                                             
        t+BiGfD2rJlCinYLgYBDpTPcdRT3VKHWqL9fcC4HKmic0mwWg9homx550wARAQAB                                                                                             
        tDFJbmZsdXhEQiBQYWNrYWdpbmcgU2VydmljZSA8c3VwcG9ydEBpbmZsdXhkYi5j                                                                                             
        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
        vqIHrEqk446nGPDqJPS5
        =9iF7
        -----END PGP PUBLIC KEY BLOCK-----
    grafana.list:
      source: "deb https://packages.grafana.com/oss/deb stable main"
      key: |
        -----BEGIN PGP PUBLIC KEY BLOCK-----
        Version: GnuPG v1
        
        mQENBFiHXVIBCADr3VDEAGpq9Sg/xrPVu1GGqWGXdbnTbbNKeveCtFHZz7/GSATW
        iwiY1skvlAOBiIKCqJEji0rZZgd8WxuhdfugiCBk1hDTMWCpjI0P+YymV77jHjYB
        jHrKNlhb+aLjEd9Gf2EtbKUT1fvGUkzlVrcRGSX/XR9MBZlgja7NIyuVbn3uwZQ4
        jflWSNSlvMpohNxTFkrBFTRrCJXhbDLfCS46+so22CP3+1VQyqJ7/6RWK9v9KYdS
        AVNgILXMggSrMqha4WA1a/ktczVQXNtP8IuPxTdp9pNYsklOTmrFVeq3mXsvWh9Q
        lIhpYHIZlTZ5wVBq4wTRchsXC5MubIhz+ASDABEBAAG0GkdyYWZhbmEgPGluZm9A
        Z3JhZmFuYS5jb20+iQE4BBMBAgAiBQJYh11SAhsDBgsJCAcDAgYVCAIJCgsEFgID
        AQIeAQIXgAAKCRCMjDTFJAmMthxJB/9Id6JrwqRkJW+eSBb71FGQmRsJvNFR8J+3
        NPVhJNkTFFOM7TnjAMUIv+LYEURqGcceTNAN1aHq/7n/8ybXucCS0CnDYyNYpyVs
        tWJ3FOQK3jPrmziDCWPQATqMM/Z2auXVFWrDFqfh2xKZNjuix0w2nyuWB8U0CG2U
        89w+ksPJblGGU5xLPPzDQoAqyZXY3gpGGTkCuohMq2RWYbp/QJSQagYhQkKZoJhr
        XJlnw4At6R1A5UUPzDw6WJqMRkGrkieE6ApIgf1vZSmnLRpXkqquRTAEyGT8Pugg
        ee6YkD19/LK6ED6gn32StY770U9ti560U7oRjrOPK/Kjp4+qBtkQuQENBFiHXVIB
        CACz4hO1g/4fKO9QWLcbSWpB75lbNgt1kHXP0UcW8TE0DIgqrifod09lC85adIz0
        zdhs+00lLqckM5wNbp2r+pd5rRaxOsMw2V+c/y1Pt3qZxupmPc5l5lL6jzbEVR9g
        ygPaE+iabTk9Np2OZQ7Qv5gIDzivqK2mRHXaHTzoQn2dA/3xpFcxnen9dvu7LCpA
        CdScSj9/UIRKk9PHIgr2RJhcjzLx0u1PxN9MEqfIsIJUUgZOoDsr8oCs44PGGIMm
        cK1CKALLLiC4ZM58B56jRyXo18MqB6VYsC1X9wkcIs72thL3tThXO70oDGcoXzoo
        ywAHBH63EzEyduInOhecDIKlABEBAAGJAR8EGAECAAkFAliHXVICGwwACgkQjIw0
        xSQJjLbWSwf/VIM5wEFBY4QLGUAfqfjDyfGXpcha58Y24Vv3n6MwJqnCIbTAaeWf
        30CZ/wHg3NNIMB7I31vgmMOEbHQdv0LPTi9TG205VQeehcpNtZRZQ0D8TIetbxyi
        Emmn9osig9U3/7jaAWBabE/9bGx4TF3eLlEH9wmFrNYeXvgRqmyqVoqhIMCNAAOY
        REYyHyy9mzr9ywkwl0aroBqhzKIPyFlatZy9oRKllY/CCKO9RJy4DZidLphuwzqU
        ymdQ1sqe5nKvwG5GvcncPc3O7LMevDBWnpNNkgERnVxCqpm90TuE3ONbirnU4+/S
        tUsVU1DERc1fjOCnAm4pKIlNYphISIE7OQ==
        =0pMC
        -----END PGP PUBLIC KEY BLOCK-----
packages:
  - mosquitto-clients
  - jq
  - grafana
  - telegraf
  - influxdb
  - net-tools
  - tree
write_files:
  - owner: root:root
    path: /etc/systemd/system/influxd.service.d/override.conf
    content: |
      [Unit]
      RequiresMountsFor=/vol

      [Service]
      Restart=always
      RestartSec=1s
  - owner: root:root
    path: /etc/systemd/system/grafana-server.service.d/override.conf
    content: |
      [Unit]
      RequiresMountsFor=/vol

      [Service]
      Restart=always
      RestartSec=1s
  - owner: root:root
    path: /etc/telegraf/telegraf.d/ttn.conf
    content: |
      [[inputs.mqtt_consumer]]
        servers = ["tcp://eu1.cloud.thethings.network:1883"]
        topics = ["v3/+/devices/+/up"]
        topic_tag = "topic"
        username = "${var.ttn-mqtt-username}"
        password = "${var.ttn-mqtt-password}"
        data_format = "json_v2"
        [[inputs.mqtt_consumer.topic_parsing]]
          topic = "+/+/+/+/+"
          tags = "_/_/_/device/_"
        [[inputs.mqtt_consumer.json_v2]]
          measurement_name = "water"
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.analog_in_5"
            rename = "ph"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.digital_in_6"
            rename = "nitrogen"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.digital_in_7"
            rename = "phosphorus"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.digital_in_8"
            rename = "potassium"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.temperature_9"
            rename = "ec"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.temperature_10"
            rename = "distance"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.temperature_11"
            rename = "temperature"
            type = "float"
            optional = true
        [[inputs.mqtt_consumer.json_v2]]
          measurement_name = "environment"
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.barometric_pressure_3"
            rename = "pressure"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.luminosity_4"
            rename = "light"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.relative_humidity_3"
            rename = "humidity"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.temperature_3"
            rename = "temperature"
            type = "float"
            optional = true
        [[inputs.mqtt_consumer.json_v2]]
          measurement_name = "position"
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.gps_1.longitude"
            rename = "longitude"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.gps_1.latitude"
            rename = "latitude"
            type = "float"
            optional = true
          [[inputs.mqtt_consumer.json_v2.field]]
            path = "uplink_message.decoded_payload.gps_1.altitude"
            rename = "altitude"
            type = "float"
            optional = true
  - owner: root:root
    path: /etc/telegraf/telegraf.d/influxdb.conf
    content: |
      [[outputs.influxdb]]
        urls = ["http://127.0.0.1:8086"]
        database = "growanywhere"
        username = "${var.influx-user-name}"
        password = "${var.influx-user-pw}"
        skip_database_creation = true
  - owner: root:root
    path: /etc/telegraf/telegraf.d/socket.conf
    content: |
      [[inputs.socket_listener]]
        service_address = "tcp://:8094"

      [[processors.override]]
        order = 1 
        namepass = ["water","environment","position"]
        fielddrop = ["*"]
        [processors.override.tagdrop]
          device = ["${var.ttn-device-id}"]
  - owner: root:root
    path: /etc/systemd/system/traefik.service
    content: |
      [Unit]
      Description=Traefik
      Documentation=https://doc.traefik.io/traefik/
      After=network-online.target
      AssertFileIsExecutable=/usr/local/bin/traefik
      AssertPathExists=/etc/traefik/traefik.toml
      RequiresMountsFor=/vol

      [Service]
      User=traefik
      CapabilityBoundingSet=CAP_NET_BIND_SERVICE
      AmbientCapabilities=CAP_NET_BIND_SERVICE
      Type=notify
      ExecStart=/usr/local/bin/traefik --configFile=/etc/traefik/traefik.toml
      Restart=always
      WatchdogSec=1s
      RestartSec=1s

      [Install]
      WantedBy=multi-user.target
  - owner: root:root
    path: /etc/traefik/traefik.toml
    content: |
      [entryPoints]
        [entryPoints.http]
        address = ":80"
        [entryPoints.https]
        address = ":443"
        [entryPoints.traefik]
        address = "127.0.0.1:8081"

      [certificatesResolvers.acmeresolver.acme]
        email = "tecer@hacknology.de"
        storage = "/etc/traefik/acme/acme.json"
        [certificatesResolvers.acmeresolver.acme.httpChallenge]
          # used during the challenge
          entryPoint = "http"

      [api]
        dashboard = true
        insecure  = true

      [providers]
        [providers.file]
        filename = "/etc/traefik/static.toml"
  - owner: root:root
    path: /etc/traefik/static.toml
    content: |
      [http]
        [http.routers]
          [http.routers.static]
          entryPoints = ["https"]
          rule = "Host(`www.${var.domain}`)"
          # middlewares = [ "extHeaders" ]
          service = "static"
          [http.routers.monitor]
          entryPoints = ["https"]
          rule = "Host(`monitor.${var.domain}`)"
          # middlewares = [ "extHeaders" ]
          service = "monitor"
          [http.routers.monitor.tls]
            certResolver = "acmeresolver"
          [http.routers.redirect]
          entryPoints = ["http"]
          rule = "HostRegexp(`.*${var.domain}`)"
          middlewares = [ "redirect" ]

        [http.middlewares]
          [http.middlewares.redirect.redirectscheme]
            scheme = "https"
          [http.middlewares.extHeaders.headers]
            [http.middlewares.extHeaders.headers.customResponseHeaders]
              "Access-Control-Allow-Origin" = "https://www.${var.domain}"
              "Access-Control-Allow-Methods" = "GET,OPTIONS,HEAD"

        [http.services]
          [http.services.monitor.loadBalancer]
            [[http.services.monitor.loadBalancer.servers]]
              url = "http://localhost:3000/"
          [http.services.static.loadBalancer]
            [[http.services.static.loadBalancer.servers]]
              url = "http://localhost:8000/"
  - owner: root:root
    path: /usr/local/bin/wait-for-mount.sh
    permissions: '0755'
    content: |
      #!/usr/bin/env bash
      while [[ ! -b "/dev/disk/by-id/scsi-0DO_Volume_${digitalocean_volume.growanywhere.name}" ]]; do
        sleep 2
      done
      sleep 2
      if mountpoint -q /vol ; then
        echo "data dir is already mounted"
      else
        echo "mounting data dir"
        # there is supposed to be a line in fstab already
        mount /vol
      fi
  - owner: root:root
    path: /usr/local/bin/move-and-link.sh
    permissions: '0755'
    content: |
      #!/usr/bin/env bash
      if [[ ! -L /etc/traefik ]]; then
        if [[ ! -d /vol/traefik ]]; then
          mv /etc/traefik /vol/
        else
          rm -rf /etc/traefik
        fi
        ln -sfn /vol/traefik /etc/traefik
      fi
      if [[ ! -L /var/lib/influxdb ]]; then
        if [[ ! -d /vol/influxdb ]]; then
          mv /var/lib/influxdb /vol/
          touch /vol/.init-influxdb
        else
          rm -rf /var/lib/influxdb
        fi
        ln -sfn /vol/influxdb /var/lib/influxdb
      fi
      if [[ ! -L /var/lib/grafana ]]; then
        if [[ ! -d /vol/grafana ]]; then
          mv /var/lib/grafana /vol/
        else
          rm -rf /var/lib/grafana
        fi
        ln -sfn /vol/grafana /var/lib/grafana
      fi
  - owner: root:root
    path: /usr/local/bin/init-influxdb.sh
    permissions: '0755'
    content: |
      #!/usr/bin/env bash
      if [[ -e /vol/.init-influxdb ]]; then
        sleep 10
        influx -execute "CREATE USER admin WITH PASSWORD '${var.influx-admin-pw}' WITH ALL PRIVILEGES"
        influx -username admin -password ${var.influx-admin-pw} -execute 'CREATE DATABASE growanywhere WITH DURATION INF SHARD DURATION 1w NAME "default"'
        influx -username admin -password ${var.influx-admin-pw} -execute "CREATE USER ${var.influx-user-name} WITH PASSWORD '${var.influx-user-pw}'"
        influx -username admin -password ${var.influx-admin-pw} -execute "GRANT ALL ON growanywhere TO ${var.influx-user-name}"
        rm -f /vol/.init-influxdb
      fi
  - owner: root:root
    path: /etc/systemd/system/growanywhere.service
    content: |
      [Unit]
      Description=Growanywhere hp + image upload
      After=network-online.target
      AssertFileIsExecutable=/usr/local/bin/growanywhere-hp
      RequiresMountsFor=/vol

      [Service]
      Type=simple
      ExecStart=/usr/local/bin/growanywhere-hp --listen :8000 --username ${var.upload-username} --password ${var.upload-password} --upload-path /vol/images
      Restart=always
      RestartSec=1s

      [Install]
      WantedBy=multi-user.target
users:
  - name: spanz
    groups: [sudo, adm]
    shell: /bin/bash
    sudo: ['ALL=(ALL) NOPASSWD:ALL']
    ssh-authorized-keys:
      - ssh-rsa AAAAB3NzaC1yc2EAAAADAQABAAABAQDEh2FeKtwy1xwBr3HtdZ7C3qrlTjXDjxaWNt8L3WWUuMgxAHWNstXctlSqodE17G7+jgDcSFB2H4vmIxWEal194tOAN0uIkpPg5JJbtV9O3csvyjlfUNmEnb9hsqNySJLEJqIuJlwqTEfoQS9V7YfY0IU/1hUqlDKUp53v+vTU48D2ZOajL776OCIqSoh3caKNbIn31pLQCqRqwn/SExgUfk7w5rJrQGR2MwR4/QDkJXesPYJ+R1pGUq+aJh9cXY5L6v8eIseoMLekDE1maBmDMHDleN7JlEbpvGmrlQUWHi/7Fno9U1xDbSsXD1tz58RZuNbdtIkIlk9qLbqa2Jj/ spanz@deskwelt
  - name: traefik
    system: true
    sudo: false
    shell: /bin/false
    no_create_home: true
    homedir: /etc/traefik
runcmd:
  - systemctl stop influxdb.service
  - systemctl stop telegraf.service
  - systemctl daemon-reload
  - /usr/local/bin/wait-for-mount.sh
  - wget -O /root/traefik_v${var.traefik-version}_linux_amd64.tar.gz https://github.com/traefik/traefik/releases/download/v${var.traefik-version}/traefik_v${var.traefik-version}_linux_amd64.tar.gz
  - mkdir -p /root/traefik
  - tar -xf /root/traefik_v${var.traefik-version}_linux_amd64.tar.gz -C /root/traefik
  - rm /root/traefik_v${var.traefik-version}_linux_amd64.tar.gz
  - mv /root/traefik/traefik /usr/local/bin/
  - rm -rf /root/traefik
  - mkdir -p /etc/traefik/acme
  - chown -R traefik:traefik /etc/traefik
  - sed -i '/auth-enabled = false/a \ \ auth-enabled = true' /etc/influxdb/influxdb.conf
  - sed -i '/cache-max-memory-size/a \ \ cache-max-memory-size = "20m"' /etc/influxdb/influxdb.conf
  - sed -i '/^;http_addr =$/a http_addr = 127.0.0.1' /etc/grafana/grafana.ini
  - sed -i '/^;allow_embedding =/a allow_embedding = true' /etc/grafana/grafana.ini
  - sed -i '/^;domain = localhost$/a domain = monitor.${var.domain}' /etc/grafana/grafana.ini
  - sed -i '/^;enforce_domain = false$/a enforce_domain = true' /etc/grafana/grafana.ini
  - sed -i '/^;root_url = /a root_url = https://monitor.${var.domain}/' /etc/grafana/grafana.ini
  - sed -i '/^;admin_password = admin$/a admin_password = ${var.grafana-admin-pw}' /etc/grafana/grafana.ini
  - sed -i '/^;allow_sign_up = true$/a allow_sign_up = false' /etc/grafana/grafana.ini
  - sed -i '/auth.anonymous/a enabled = true' /etc/grafana/grafana.ini
  - sed -i '/^;org_name = Main Org.$/a org_name = Grow Everywhere' /etc/grafana/grafana.ini
  - sed -i '/^;org_role = Viewer$/a org_role = Viewer' /etc/grafana/grafana.ini
  - sed -i '/^;hide_version = false$/a hide_version = true' /etc/grafana/grafana.ini
  - sed -i '/auth.basic/a enabled = false' /etc/grafana/grafana.ini
  - /usr/local/bin/move-and-link.sh
  - systemctl start influxdb.service
  - /usr/local/bin/init-influxdb.sh
  - systemctl start telegraf.service
  - systemctl enable grafana-server.service
  - systemctl start grafana-server.service
  - systemctl enable traefik.service
  - systemctl start traefik.service
EOF
}