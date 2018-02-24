#include <arv.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h>

typedef struct {
    GMainLoop *main_loop;
    int buffer_count;
} ApplicationData;

static gboolean cancel = FALSE;

static void
set_cancel (int signal)
{
    cancel = TRUE;
}

static void
new_buffer_cb (ArvStream *stream, ApplicationData *data)
{
    ArvBuffer *buffer;
    buffer = arv_stream_try_pop_buffer (stream);
    if (buffer != NULL) {
        if (arv_buffer_get_status (buffer) == ARV_BUFFER_STATUS_SUCCESS){
          printf ("Frame ID: %d, %ld\t", arv_buffer_get_frame_id(buffer), arv_buffer_get_timestamp(buffer));
          if(ARV_BUFFER_PAYLOAD_TYPE_IMAGE == arv_buffer_get_payload_type (buffer))
          {
            printf ("Image received \t %x \n", arv_buffer_get_image_pixel_format(buffer));
          }
        }
          data->buffer_count++;
        /* Image processing here */
        arv_stream_push_buffer (stream, buffer);
    }
}

static gboolean
periodic_task_cb (void *abstract_data)
{
    ApplicationData *data = static_cast<ApplicationData*>(abstract_data);

    printf ("Frame rate = %d Hz\n", data->buffer_count);
    data->buffer_count = 0;

    if (cancel) {
        g_main_loop_quit (data->main_loop);
        return FALSE;
    }

    return TRUE;
}

static void
control_lost_cb (ArvGvDevice *gv_device)
{
    /* Control of the device is lost. Display a message and force application exit */
    printf ("Control lost\n");

    cancel = TRUE;
}

int
main (int argc, char **argv)
{
    ApplicationData data;
    ArvCamera *camera;
    ArvStream *stream;
    int i;

    data.buffer_count = 0;

    /* Mandatory glib type system initialization */
    arv_g_type_init ();

    /* Instantiation of the first available camera */
    camera = arv_camera_new (NULL);

    if (camera != NULL) {
      printf ("Cam Opened: %s\n", arv_camera_get_device_id(camera));
      ArvDevice * device = arv_camera_get_device (camera);

      if(device){
        guint32 ip_addr = arv_device_get_integer_feature_value (device, "GevPersistentIPAddress");
        printf("PIP: %x\t ", arv_device_get_integer_feature_value (device, "GevPersistentIPAddress"));
        printf("IP: %d.%d.%d.%d\n", ((ip_addr&0xff000000)>>24), ((ip_addr&0x00ff0000)>>16), 
                ((ip_addr&0x0000ff00)>>8), (ip_addr&0x000000ff));
        guint32 subnet_mask = arv_device_get_integer_feature_value (device, "GevPersistentSubnetMask");
        printf("PMask: %x\t ", arv_device_get_integer_feature_value (device, "GevPersistentSubnetMask"));
        printf("Subnet Mask: %d.%d.%d.%d\n", ((subnet_mask&0xff000000)>>24), ((subnet_mask&0x00ff0000)>>16), 
                ((subnet_mask&0x0000ff00)>>8), (subnet_mask&0x000000ff));
      }
              
      //arv_device_set_integer_feature_value(device, "GevPersistentIPAddress", 0xc0a80128);
      //printf("PIP: %x\n ", arv_device_get_integer_feature_value (device, "GevPersistentIPAddress"));
      
      //arv_device_set_integer_feature_value(device, "GevPersistentSubnetMask", 0xffffff00);
      //printf("PMask: %x\n ", arv_device_get_integer_feature_value (device, "GevPersistentSubnetMask"));
      
      //arv_device_set_boolean_feature_value(device, "GevCurrentIPConfigurationPersistentIP", 1);
      //printf("PIPFlag: %x\n ", arv_device_get_boolean_feature_value (device, "GevCurrentIPConfigurationPersistentIP"));
      
      //arv_device_set_boolean_feature_value(device, "GevCurrentIPConfigurationDHCP", 0);
      //printf("DHCPFlag: %x\n ", arv_device_get_boolean_feature_value (device, "GevCurrentIPConfigurationDHCP"));
      
      
        void (*old_sigint_handler)(int);
        gint payload;

        /* Set region of interrest to a 200x200 pixel area */
        arv_camera_set_region (camera, 0, 0, 200, 200);
        /* Set frame rate to 10 Hz */
        arv_camera_set_frame_rate (camera, 30.0);
        /* retrieve image payload (number of bytes per image) */
        payload = arv_camera_get_payload (camera);

        /* Create a new stream object */
        stream = arv_camera_create_stream (camera, NULL, NULL);
        if (stream != NULL) {
            /* Push 50 buffer in the stream input buffer queue */
            for (i = 0; i < 50; i++)
                arv_stream_push_buffer (stream, arv_buffer_new (payload, NULL));

            /* Start the video stream */
            arv_camera_start_acquisition (camera);

            /* Connect the new-buffer signal */
            g_signal_connect (stream, "new-buffer", G_CALLBACK (new_buffer_cb), &data);
            /* And enable emission of this signal (it's disabled by default for performance reason) */
            arv_stream_set_emit_signals (stream, TRUE);

            /* Connect the control-lost signal */
            g_signal_connect (arv_camera_get_device (camera), "control-lost",
                      G_CALLBACK (control_lost_cb), NULL);

            /* Install the callback for frame rate display */
            g_timeout_add_seconds (1, periodic_task_cb, &data);

            /* Create a new glib main loop */
            data.main_loop = g_main_loop_new (NULL, FALSE);

            old_sigint_handler = signal (SIGINT, set_cancel);
            printf ("Frame rate set = %f Hz\n", arv_camera_get_frame_rate (camera));
            printf ("Cam Vendor: %s\n", arv_camera_get_vendor_name (camera));
            printf ("Cam Model: %s\n", arv_camera_get_model_name (camera));
            if(arv_camera_is_gv_device(camera)){
              printf ("GigE\n"); 
            }
            /* Run the main loop */
            g_main_loop_run (data.main_loop);

            signal (SIGINT, old_sigint_handler);

            g_main_loop_unref (data.main_loop);

            /* Stop the video stream */
            arv_camera_stop_acquisition (camera);

            g_object_unref (stream);
        } else
            printf ("Can't create stream thread (check if the device is not already used)\n");

        g_object_unref (camera);
    } else
        printf ("No camera found\n");

    return 0;
}
