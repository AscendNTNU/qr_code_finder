def getRepoURL() {
  sh "git config --get remote.origin.url > .git/remote-url"
  return readFile(".git/remote-url").trim()
}

def getCommitSha() {
  sh "git rev-parse HEAD > .git/current-commit"
  return readFile(".git/current-commit").trim()
}

void setBuildStatus(String message, String state) {
  step([
      $class: "GitHubCommitStatusSetter",
      reposSource: [$class: "ManuallyEnteredRepositorySource", url: getRepoURL()],
      contextSource: [$class: "ManuallyEnteredCommitContextSource", context: "Jenkins Pipeline"],
      errorHandlers: [[$class: "ChangingBuildStatusErrorHandler", result: "UNSTABLE"]],
      statusResultSource: [ $class: "ConditionalStatusResultSource", results: [[$class: "AnyBuildResult", message: message, state: state]] ]
  ]);
}



pipeline {
    agent any
    stages {
        stage('Compilation') {
            steps {
                sh "docker build -t ${env.JOB_NAME}:${env.GIT_COMMIT} ."
            }
        }
        // stage('Unit Tests') {
        //     steps {
        //         sh "docker run ${env.JOB_NAME}:${env.GIT_COMMIT} /ros_entrypoint.sh tests"
        //     }
        // }
    }
    post {
        success {
            setBuildStatus("Build succeeded", "SUCCESS");
        }
        failure {
            setBuildStatus("Build failed", "FAILURE");
        }
    }
}