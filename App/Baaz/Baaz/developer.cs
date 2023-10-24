using Android.App;
using Android.OS;
using Android.Widget;
using AndroidX.AppCompat.App;

namespace Baaz
{
    [Activity(Label = "developer Activity")]
    public class developerActivity : AppCompatActivity
    {
        protected override void OnCreate(Bundle savedInstanceState)
        {
            base.OnCreate(savedInstanceState);
            SetContentView(Resource.Layout.developer);
            // Additional initialization code for HomeActivity, if needed.
            Button buttonHome = FindViewById<Button>(Resource.Id.button1);
            Button buttonConnect = FindViewById<Button>(Resource.Id.button2);
            Button buttonMapping = FindViewById<Button>(Resource.Id.button3);
            Button buttonSupport = FindViewById<Button>(Resource.Id.button4);

            buttonHome.Click += (sender, e) =>
            {
                StartActivity(new Android.Content.Intent(this, typeof(MainActivity)));
            };

            buttonConnect.Click += (sender, e) =>
            {
                StartActivity(new Android.Content.Intent(this, typeof(connectActivity)));
            };

            buttonMapping.Click += (sender, e) =>
            {
                StartActivity(new Android.Content.Intent(this, typeof(mappingActivity)));
            };

            buttonSupport.Click += (sender, e) =>
            {
                StartActivity(new Android.Content.Intent(this, typeof(supportActivity)));
            };
        }
    }
}
