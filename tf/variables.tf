variable "domain" {
  default = "growanywhere.de"
}

variable "region" {
  default = "fra1"
}

variable "size" {
  default = "s-1vcpu-512mb-10gb" // "s-1vcpu-1gb"
}

variable "volume-size" {
  default = 10
}

variable "traefik-version" {
  default = "2.8.1"
}

variable "grafana-admin-pw" {
  sensitive = true
}

variable "influx-admin-pw" {
  sensitive = true
}

variable "influx-user-name" {
  default = "growanywhere"
}

variable "influx-user-pw" {
  sensitive = true
}

variable "ttn-mqtt-username" {
  default = "growanywhere@ttn"
}

variable "ttn-mqtt-password" {
  sensitive = true
}

variable "ttn-device-id" {
  sensitive = true
}