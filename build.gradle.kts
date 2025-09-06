// Top-level build file where you can add configuration options common to all sub-projects/modules.
plugins {
    alias(libs.plugins.android.application) apply false
    alias(libs.plugins.kotlin.android) apply false
    alias(libs.plugins.kotlin.compose) apply false
}

project.delete {
    delete(rootProject.layout.buildDirectory)
}

subprojects {
    tasks.withType(type = Test::class) {
        // Gradle 5+ reduced default heap and permgen sizes.
        // Robolectric and mockkk/mockito need more memory during test execution.
        maxParallelForks = 2
        forkEvery = 80
        maxHeapSize = "2048m"
        minHeapSize = "1024m"
    }
}