pipeline {
    agent { label 'server28' }
    
    environment {
        GIT_BRANCH_DOCKER = 'adas'
        GIT_BRANCH_PROJECT = "${(env.GIT_LOCAL_BRANCH ?: env.GIT_BRANCH)?.replaceAll('^origin/', '') ?: 'master'}"
        IMAGE_NAME = 'adas_sdk_image'
        CONTAINER_NAME = "${GIT_BRANCH_PROJECT.toLowerCase().replaceAll('[^a-z0-9-_.]', '_')}_container"

        OUTPUT_FORMAT_COBERTUNA_REPORT = 'xml'
        TYPE_UNIT_TEST = 'unit-test'
        TYPE_INTEGRATION_TEST = 'integration-test'
    }

    stages {
        stage('Docker') {
            steps {
                echo 'Docker Building...'
                script {
                    withCredentials([usernamePassword(credentialsId: 'jenkins_adas', usernameVariable: 'GIT_USERNAME', passwordVariable: 'GIT_PASSWORD')]) {
                        // 1. Clone repository
                        withCredentials([usernamePassword(credentialsId: 'docker_adas', usernameVariable: 'GIT_USERNAME_DOCKER', passwordVariable: 'GIT_PASSWORD_DOCKER')]) {
                            // git branch: "${GIT_BRANCH_DOCKER}", credentialsId: 'docker_adas', url: 'https://gitlab-scm.banvien.com.vn/playground/adas-hackathon1/development/docker.git'
                            sh """
                                # Remove repo Docker
                                if [ -d "docker" ]; then
                                    rm -rf docker
                                fi
                                
                                # Clone repo Docker
                                git clone https://${GIT_USERNAME_DOCKER}:${GIT_PASSWORD_DOCKER}@gitlab-scm.banvien.com.vn/playground/adas-hackathon1/development/docker.git -b ${GIT_BRANCH_DOCKER}
                            """
                        }
                        
                        /*  2. Build Docker image
                            Arguments used:
                            - GITLAB_USERNAME: GitLab username for repo access
                            - GITLAB_TOKEN: GitLab access token for secure repo access
                            - GIT_BRANCH: Branch to clone for build
                            - CACHE_BUST: Timestamp to refresh Docker cache
                        */
                        sh '''
                            cd docker

                            docker build \
                                --build-arg GITLAB_USERNAME=${GIT_USERNAME} \
                                --build-arg GITLAB_TOKEN=${GIT_PASSWORD} \
                                --build-arg GIT_BRANCH=${GIT_BRANCH_PROJECT} \
                                --build-arg CACHE_BUST=$(date +%s) \
                                --network="host" \
                                -t ${IMAGE_NAME} .
                        '''
                        
                        // 3. Run container
                        // - Check if container exists, remove if it does, then run a new container
                        sh '''
                            docker run --name ${CONTAINER_NAME} -d ${IMAGE_NAME} tail -f /dev/null
                        '''
                    }
                }
            }
        }

        stage('Static Check') {
            steps {
                sh """
                    docker exec ${CONTAINER_NAME} ./script/static_analysis_scripts/run_static_analysis_scripts.sh
                    docker cp ${CONTAINER_NAME}:/adas_sdk/cppcheck_report/cppcheck_results.xml ./cppcheck_results.xml
                """

                // Publish Cppcheck Report by Plugin of Jenkins
                publishCppcheck(
                    pattern: 'cppcheck_results.xml',
                    healthy: '0',
                    unHealthy: '150', 
                    threshold: 'high',

                    severityInformation: false,
                    severityNoCategory: false,
                    severityPortability: true,
                    severityError: true,
                    severityWarning: true,
                    severityStyle: true,
                    severityPerformance: true
                )
            }
        }

        stage('Build') {
            steps {
                echo 'Building...'
                // 1. Build project
                sh '''
                docker exec ${CONTAINER_NAME} bash -c "
                    mkdir -p pc_build
                    cd pc_build
                    export PC_BUILD=true
                    cmake -DUT_TEST=ON ..
                    make -j4
                "
                '''
            }
        }

        stage('Unit Test') {
            steps {
                echo 'Unit Test...'
                script{
                    // 1. Build and run unit test project
                    sh '''
                        docker exec ${CONTAINER_NAME} bash -c "
                            ./script/test_scripts/run_test_scripts.sh --mode=build --type=${TYPE_UNIT_TEST}
                            ./script/test_scripts/run_test_scripts.sh --mode=run --type=${TYPE_UNIT_TEST}
                        "
                    '''
                }
            }
        }

        stage('Integration Test') {
            steps {
                echo 'IT...'
            }
        }

        stage('Unit Test Coverage') {
            steps {
                echo 'Unit Test Coverage...'
                
                sh '''
                    docker exec ${CONTAINER_NAME} bash -c "
                        ./script/test_scripts/generate_test_scripts.sh --output_format=${OUTPUT_FORMAT_COBERTUNA_REPORT}
                    "
                    docker cp ${CONTAINER_NAME}:/adas_sdk/adas_service/summary_report ./adas_service
                '''
            }
        }
    }
    post {
        always {
            // Archive the coverage report
            cobertura coberturaReportFile: 'adas_service/summary_report/summary_report.xml'

            // Clean-up: Remove the container
            sh '''
                if [ "$(docker ps -aq -f name=${CONTAINER_NAME})" ]; then
                    docker rm -f ${CONTAINER_NAME}
                fi
            '''
        }
    }
}