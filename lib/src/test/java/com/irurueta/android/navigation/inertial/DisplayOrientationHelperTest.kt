package com.irurueta.android.navigation.inertial

import android.content.Context
import android.os.Build
import android.view.Display
import android.view.Surface
import android.view.WindowManager
import androidx.test.core.app.ApplicationProvider
//import io.mockk.*
//import io.mockk.impl.annotations.MockK
//import io.mockk.impl.annotations.SpyK
//import io.mockk.junit4.MockKRule
//import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Before
//import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.Mock
import org.mockito.Spy
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.spy
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

//@Ignore("Possible memory leak when running this test")
@Suppress("DEPRECATION")
@RunWith(RobolectricTestRunner::class)
class DisplayOrientationHelperTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK
    @Mock
    private lateinit var display: Display

//    @MockK
    @Mock
    private lateinit var windowManager: WindowManager

    @Mock
    private lateinit var context: Context

//    @SpyK
//    @Spy
//    private lateinit var context: Context // = ApplicationProvider.getApplicationContext()

/*    @Before
    fun setup() {
        context = spy(ApplicationProvider.getApplicationContext<Context>())
    }*/

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

//    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAndZeroRotation_returns0() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_0)
//        every { display.rotation }.returns(Surface.ROTATION_0)
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

//    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAnd90Rotation_returns90() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_90)
//        every { display.rotation }.returns(Surface.ROTATION_90)
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)

        assertEquals(90.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

//    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAnd180Rotation_returns180() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_180)
//        every { display.rotation }.returns(Surface.ROTATION_180)
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)

        assertEquals(180.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

//    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationDegrees_whenSdkRAnd270Rotation_returns180() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_270)
//        every { display.rotation }.returns(Surface.ROTATION_270)
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)

        assertEquals(270.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAndZeroRotation_returns0() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_0)
//        every { display.rotation }.returns(Surface.ROTATION_0)
        whenever(windowManager.defaultDisplay).thenReturn(display)
//        every { windowManager.defaultDisplay }.returns(display)
        doReturn(windowManager).whenever(context).getSystemService(Context.WINDOW_SERVICE)
//        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAnd90Rotation_returns90() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_90)
//        every { display.rotation }.returns(Surface.ROTATION_90)
        doReturn(display).whenever(windowManager).defaultDisplay
//        every { windowManager.defaultDisplay }.returns(display)
        doReturn(windowManager).whenever(context).getSystemService(Context.WINDOW_SERVICE)
//        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(90.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAnd180Rotation_returns180() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_180)
//        every { display.rotation }.returns(Surface.ROTATION_180)
        doReturn(display).whenever(windowManager).defaultDisplay
//        every { windowManager.defaultDisplay }.returns(display)
        doReturn(windowManager).whenever(context).getSystemService(Context.WINDOW_SERVICE)
//        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(180.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationDegrees_whenSdkQAnd270Rotation_returns270() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_270)
//        every { display.rotation }.returns(Surface.ROTATION_270)
        doReturn(display).whenever(windowManager).defaultDisplay
//        every { windowManager.defaultDisplay }.returns(display)
        doReturn(windowManager).whenever(context).getSystemService(Context.WINDOW_SERVICE)
//        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(270.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

//    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAndZeroRotation_returns0() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_0)
//        every { display.rotation }.returns(Surface.ROTATION_0)
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationDegrees(context), 0.0)
    }

//    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAnd90Rotation_returnsPiDividedBy2() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_90)
//        every { display.rotation }.returns(Surface.ROTATION_90)
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)

        assertEquals(
            Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }

//    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAnd180Rotation_returnsPi() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_180)
//        every { display.rotation }.returns(Surface.ROTATION_180)
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)

        assertEquals(Math.PI, DisplayOrientationHelper.getDisplayRotationRadians(context), 0.0)
    }

//    @Config(sdk = [Build.VERSION_CODES.R])
    @Test
    fun getDisplayRotationRadians_whenSdkRAnd270Rotation_returns3PiDividedBy2() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_270)
//        every { display.rotation }.returns(Surface.ROTATION_270)
        doReturn(display).whenever(context).display
//        every { context.display }.returns(display)

        assertEquals(
            3.0 * Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAndZeroRotation_returns0() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_0)
//        every { display.rotation }.returns(Surface.ROTATION_0)
        whenever(windowManager.defaultDisplay).thenReturn(display)
//        every { windowManager.defaultDisplay }.returns(display)
        doReturn(windowManager).whenever(context).getSystemService(Context.WINDOW_SERVICE)
//        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(0.0, DisplayOrientationHelper.getDisplayRotationRadians(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAnd90Rotation_returnsPiDividedBy2() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_90)
//        every { display.rotation }.returns(Surface.ROTATION_90)
        whenever(windowManager.defaultDisplay).thenReturn(display)
//        every { windowManager.defaultDisplay }.returns(display)
        doReturn(windowManager).whenever(context).getSystemService(Context.WINDOW_SERVICE)
//        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(
            Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAnd180Rotation_returnsPi() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_180)
//        every { display.rotation }.returns(Surface.ROTATION_180)
        whenever(windowManager.defaultDisplay).thenReturn(display)
//        every { windowManager.defaultDisplay }.returns(display)
        doReturn(windowManager).whenever(context).getSystemService(Context.WINDOW_SERVICE)
//        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(Math.PI, DisplayOrientationHelper.getDisplayRotationRadians(context), 0.0)
    }

    @Config(sdk = [Build.VERSION_CODES.Q])
    @Test
    fun getDisplayRotationRadians_whenSdkQAnd270Rotation_returns3PiDividedBy2() {
        whenever(display.rotation).thenReturn(Surface.ROTATION_270)
//        every { display.rotation }.returns(Surface.ROTATION_270)
        whenever(windowManager.defaultDisplay).thenReturn(display)
//        every { windowManager.defaultDisplay }.returns(display)
        doReturn(windowManager).whenever(context).getSystemService(Context.WINDOW_SERVICE)
//        every { context.getSystemService(Context.WINDOW_SERVICE) }.returns(windowManager)

        assertEquals(
            3.0 * Math.PI / 2.0,
            DisplayOrientationHelper.getDisplayRotationRadians(context),
            0.0
        )
    }
}