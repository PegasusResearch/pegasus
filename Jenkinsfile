// Developed by: Marcelo Jacinto
// Date: 01/06/2022
pipeline {
    agent {
        docker {
            image 'docker.io/dsorisr/pegasus:v0.0.1'
            args '--entrypoint=""'
            reuseNode false
        }
    }
    environment {
        ROS_DISTRO="foxy"
    }
    options {
        checkoutToSubdirectory('pegasus_ws/src')
    }
    stages {
        stage('Build') {
            steps {
                echo 'Building...'
                dir('pegasus_ws') {
                    sh'''#!/bin/bash
                    source /opt/ros/${ROS_DISTRO}/setup.bash
                    '''
                }
            }
        }
    }
}