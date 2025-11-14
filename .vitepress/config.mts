import { defineConfig } from 'vitepress'

// https://vitepress.dev/reference/site-config
export default defineConfig({
  title: "Software Stack Documentation",
  description: "This documentation is the master entry point for all things DroneLeaf.",
  themeConfig: {
    // https://vitepress.dev/reference/default-theme-config
    search: {
      provider: 'local'
    },
    nav: [
      // { text: 'Home', link: '/' },
      { text: 'README', link: '/README.html' }
    ],

    sidebar: [
      {
        text: 'README Contents',
        items: [
          { text: 'Home', link: '/README.html' },
          {
            text: 'Anatomy of the software stack',
            items: [
              { text: 'Anatomy overview', link: '/README.html#anatomy-of-the-software-stack' }
            ]
          },
          {
            text: 'Definitions & Tools',
            items: [
              { text: 'Definitions of used terms', link: '/README.html#definitions-of-used-terms' },
              { text: 'Additional tools and directories', link: '/README.html#additional-tools-and-directories' }
            ]
          },
          {
            text: 'Getting Started',
            items: [
              { text: 'Deployment (clients)', link: '/README.html#getting-started-with-deployment-targeted-for-droneleaf-clients' },
              {
                text: 'Development',
                items: [
                  { text: 'Overview', link: '/README.html#getting-started-with-development' },
                  {
                    text: 'Preparing the Developer Machine',
                    items: [
                      { text: 'Prerequisites', link: '/README.html#prerequisites' },
                      { text: 'OS Installation', link: '/README.html#os-installation' },
                      { text: 'Tools and Packages Installation', link: '/README.html#tools-and-packages-installation' },
                      { text: 'HEAR-CLI', link: '/README.html#hear-cli' }
                    ]
                  }
                ]
              }
            ]
          },
          {
            text: 'Flight Stack',
            items: [
              { text: 'Getting started with Flight Stack Development', link: '/README.html#getting-started-with-flight-stack-development' },
              { text: 'Cloning', link: '/README.html#cloning' },
              {
                text: 'Compilation',
                items: [
                  { text: 'HEAR_Msgs', link: '/README.html#hear_msgs' },
                  { text: 'LeafFC', link: '/README.html#leaffc' },
                  { text: 'PX4 Autopilot', link: '/README.html#px4-autopilot' },
                  { text: 'LeafMC', link: '/README.html#leafmc' }
                ]
              }
            ]
          },
          {
            text: 'Petals & Apps',
            items: [
              { text: 'Petals stack', link: '/README.html#getting-started-with-petals-stack-development' },
              { text: 'Petal App Manager quickstart', link: '/README.html#getting-started-with-petals-stack-development' }
            ]
          },
          { text: 'Commissioning', link: '/README.html#commissioning' },
          {
            text: 'Running SITL / Bench',
            items: [
              { text: 'Overview', link: '/README.html#running-the-sitlbench-environment' },
              { text: 'Build and launch PX4', link: '/README.html#1-build-and-launch-px4-gazebo-classic' },
              { text: 'Start mavlink-router', link: '/README.html#2-start-the-mavlink-router-service' },
              { text: 'Launch HEAR_FC', link: '/README.html#3-launch-hear_fc' },
              { text: 'Launch LeafMC', link: '/README.html#4-launch-leafmc' },
              { text: 'Petal App Manager', link: '/README.html#5-petal-app-manager' }
            ]
          },
          {
            text: 'Debugging & Tools',
            items: [
              { text: 'Debugging Tools', link: '/README.html#debugging-tools' },
              { text: 'Debugging MAVLink with Wireshark', link: '/README.html#debugging-mavlink-with-wireshark' }
            ]
          },
          { text: 'Additional Functionalities: VPN', link: '/README.html#vpn-remote-access' },
          { text: 'Contribution and development', link: '/README.html#contribution-and-development' },
          { text: 'Known Issues', link: '/README.html#known-issues' }
        ]
      },
      {
        text: 'Guides',
        items: [
          {
            text: 'Development Machine',
            items: [
              { text: 'Ubuntu 20.04 install steps', link: '/Guide/Hardware%20and%20Process/Development%20Machine%20Preparation/installation_steps_for_Ubuntu_20.04_LTS.html' },
              { text: 'VPN remote access', link: '/Guide/Hardware%20and%20Process/Development%20Machine%20Preparation/vpn_remote_access_setup.html' }
            ]
          },
          {
            text: 'External Software',
            items: [
              { text: 'VSCode guide', link: '/Guide/External%20Software/VSCode/README.html' },
              { text: 'Yakuake guide', link: '/Guide/External%20Software/Yakuake/README.html' },
              { text: 'MAVLink / Wireshark debugging', link: '/Guide/External%20Software/MAVLink/mavlink_debugging.html' }
            ]
          },
          {
            text: 'HEAR Software',
            items: [
              { text: 'HEAR_CLI (setup)', link: '/Guide/HEAR%20Software/HEAR_CLI/README.html' },
              { text: 'HEAR-CLI development guide', link: '/Guide/HEAR%20Software/Development/hear-cli-development-guide.html' },
              { text: 'LeafMC guide', link: '/Guide/HEAR%20Software/LeafMC/README.html' },
              { text: 'LeafMC & Qt tooling', link: '/Guide/HEAR%20Software/Operation/SITL/leafQGC-and-QT-tooling.html' },
              { text: 'leafFC debugging (DynamoDB)', link: '/Guide/HEAR%20Software/Operation/SITL/DynamoDB-and-hearfc-debugging.html' },
              { text: 'Software development & releasing workflow', link: '/Guide/HEAR%20Software/Development%20Workflow/software_development_and_releasing_workflow.html' }
            ]
          },
          {
            text: 'Operation / SITL',
            items: [
              { text: 'PX4 / SITL install', link: '/Guide/HEAR%20Software/Operation/SITL/sitl-installation-on-ubuntu20.04.html' },
              { text: 'Quick Commissioning Guide', link: '/Guide/HEAR%20Software/Operation/SITL/quick_commissioning_guide.html' },
              { text: 'Quick Commissioning Guide', link: '/Guide/HEAR%20Software/Operation/SITL/quick_commissioning_guide.html' },
              { text: 'Known Issues', link: '/Guide/HEAR%20Software/Operation/SITL/known_issues.html' }
            ]
          }
        ]
      }
    ],

    socialLinks: [
      { icon: 'github', link: 'https://github.com/DroneLeaf' }
    ]
  }
})
