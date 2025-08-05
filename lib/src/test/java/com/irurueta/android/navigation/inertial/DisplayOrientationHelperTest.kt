package com.irurueta.android.navigation.inertial

import android.content.Context
import android.os.Build
import android.view.Display
import android.view.Surface
import android.view.WindowManager
import androidx.test.core.app.ApplicationProvider
import io.mockk.*
import io.mockk.impl.annotations.MockK
import io.mockk.impl.annotations.SpyK
import io.mockk.junit4.MockKRule
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

@Ignore("possible memory leak")
@Suppress("DEPRECATION")
@RunWith(RobolectricTestRunner::class)
class DisplayOrientationHelperTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var display: Display

    @MockK
    private lateinit var windowManager: WindowManager

    @SpyK
    private var context: Context = ApplicationProvider.getApplicationContext()

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAndZeroRotation_returns0() {
        every { display.rotation }.returns(Surface.ROTATION_0)
        every { context.display }.returns(display)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAnd90Rotation_returns90() {
        every { display.rotation }.returns(Surface.ROTATION_90)
        every { context.display }.returns(display)

        assertEquals(90.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAnd180Rotation_returns180() {
        every { display.rotation }.returns(Surface.ROTATION_180)
        every { context.display }.returns(display)

        assertEquals(180.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAnd270Rotation_returns180() {
        every { display.rotation }.returns(Surface.ROTATION_270)
        every { context.display }.returns(display)

        assertEquals(270.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAndZeroRotation_returns0() {
        every { display.rotation }.returns(Surface.ROTATION_0)
        every { windowManager.defaultDisplay }.returns(display)
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAnd90Rotation_returns90() {
        every { display.rotation }.returns(Surface.ROTATION_90)
        every { windowManager.defaultDisplay }.returns(display)
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(90.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAnd180Rotation_returns180() {
        every { display.rotation }.returns(Surface.ROTATION_180)
        every { windowManager.defaultDisplay }.returns(display)
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(180.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAnd270Rotation_returns270() {
        every { display.rotation }.returns(Surface.ROTATION_270)
        every { windowManager.defaultDisplay }.returns(display)
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(270.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAndZeroRotation_returns0() {
        every { display.rotation }.returns(Surface.ROTATION_0)
        every { context.display }.returns(display)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAnd90Rotation_returnsPiDividedBy2() {
        every { display.rotation }.returns(Surface.ROTATION_90)
        every { context.display }.returns(display)

        assertEquals(
            Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAnd180Rotation_returnsPi() {
        every { display.rotation }.returns(Surface.ROTATION_180)
        every { context.display }.returns(display)

        assertEquals(Math.PI, DisplayOrientationHelper.getDisplayRotationRadians(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAnd270Rotation_returns3PiDividedBy2() {
        every { display.rotation }.returns(Surface.ROTATION_270)
        every { context.display }.returns(display)

        assertEquals(
            3.0 * Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAndZeroRotation_returns0() {
        every { display.rotation }.returns(Surface.ROTATION_0)
        every { windowManager.defaultDisplay }.returns(display)
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationRadians(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAnd90Rotation_returnsPiDividedBy2() {
        every { display.rotation }.returns(Surface.ROTATION_90)
        every { windowManager.defaultDisplay }.returns(display)
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(
            Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAnd180Rotation_returnsPi() {
        every { display.rotation }.returns(Surface.ROTATION_180)
        every { windowManager.defaultDisplay }.returns(display)
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(Math.PI, DisplayOrientationHelper.getDisplayRotationRadians(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAnd270Rotation_returns3PiDividedBy2() {
        every { display.rotation }.returns(Surface.ROTATION_270)
        every { windowManager.defaultDisplay }.returns(display)
        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(
            3.0 * Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }
}