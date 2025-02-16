def update_particles_with_motion_mode(self, motion_mode, twist, dt):
    # update particle poses based on the motion mode

    for p in self.particle_cloud:
        # calculate movement based on twist and motion mode
        if motion_mode == "straight":
            p.pose.position.x += twist.linear.x * math.cos(get_yaw_from_pose(p.pose)) * dt
            p.pose.position.y += twist.linear.x * math.sin(get_yaw_from_pose(p.pose)) * dt
        elif motion_mode == "rotate":
            p.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, get_yaw_from_pose(p.pose) + twist.angular.z * dt))
        elif motion_mode == "curve":
            R = twist.linear.x / twist.angular.z
            cx = p.pose.position.x - (R + self.robot_radius) * math.sin(get_yaw_from_pose(p.pose))
            cy = p.pose.position.y + (R + self.robot_radius) * math.cos(get_yaw_from_pose(p.pose))
            p.pose.position.x = cx + (R + self.robot_radius) * math.sin(get_yaw_from_pose(p.pose) + twist.angular.z * dt)  
            p.pose.position.y = cy - (R + self.robot_radius) * math.cos(get_yaw_from_pose(p.pose) + twist.angular.z * dt)  
            p.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, get_yaw_from_pose(p.pose) + twist.angular.z * dt))

    def update_particle_weights_with_measurement_model(self, data):
        for p in self.particle_cloud:
            # get the pose of the particle
            x = p.pose.position.x
            y = p.pose.position.y
            theta = get_yaw_from_pose(p.pose)

            # calculate the weight of the particle using the measurement model
            w = 1.0
            for i in range(0, len(data.ranges), 10):
                # only consider beams with finite range
                if data.ranges[i] < data.range_max:
                    # calculate the angle of the ray with respect to the robot
                    angle = data.angle_min + i * data.angle_increment
                    # calculate the position of the hit point in the robot frame
                    x_hit = data.ranges[i] * math.cos(angle)
                    y_hit = data.ranges[i] * math.sin(angle)

                    # transform the hit point to the map frame
                    x_map = x + math.cos(theta) * x_hit - math.sin(theta) * y_hit
                    y_map = y + math.sin(theta) * x_hit + math.cos(theta) * y_hit

                    # get the closest obstacle distance at the hit point in the map frame
                    dist = self.likelihood_field.get_closest_obstacle_distance(x_map, y_map)
                    # calculate the weight of the particle using the beam likelihood field
                    w *= self.likelihood_field.get_beam_likelihood_field_value(dist)

            # update the weight of the particle
            p.w = w

        # normalize the weights of the particles
        weights = [p.w for p in self.particle_cloud]
        sum_weights = sum(weights)
        for p in self.particle_cloud:
            p.w /= sum_weights

        # Resample the particle cloud
        self.resample_particles()

        def update_particle_weights_with_measurement_model(self, data):
            # measurement model: compute the weight of each particle
            # according to the probability of the laser scan data given the particle's pose
            # (Equation 1 in the homework instruction)

            # likelihood field initialization
            if not self.init_map:
                return
            lf = LikelihoodField(self.map)

            # update particle weights
            for particle in self.particle_cloud:
                # get particle pose in the map frame
                x, y, theta = particle.pose.position.x, particle.pose.position.y, get_yaw_from_pose(particle.pose)

                # compute expected ranges
                expected_ranges = []
                for i in range(len(data.ranges)):
                    angle = data.angle_min + i * data.angle_increment
                    x_laser = x + data.ranges[i] * math.cos(angle + theta)
                    y_laser = y + data.ranges[i] * math.sin(angle + theta)
                    expected_range = lf.get_closest_obstacle_distance(x_laser, y_laser)
                    expected_ranges.append(expected_range)

                # compute weight
                particle.w = np.prod(
                    np.exp(-(np.array(data.ranges) - np.array(expected_ranges)) ** 2 / (2 * self.sigma ** 2)))

            # normalize weights
            weights = np.array([particle.w for particle in self.particle_cloud])
            weights /= np.sum(weights)

            # resample particles
            new_particle_cloud = []
            indices = np.random.choice(self.num_particles, self.num_particles, p=weights)
            for index in indices:
                new_particle_cloud.append(copy.deepcopy(self.particle_cloud[index]))
            self.particle_cloud = new_particle_cloud


def gaussian_probability(x, mean, sd):
    """
    使用高斯分布計算機率的函數。

    :param x: 事件的觀察值
    :param mean: 高斯分布的平均值
    :param sd: 高斯分布的標準差
    :return: 機率的浮點數值
    """
    exponent = math.exp(-((x - mean) ** 2 / (2 * sd ** 2)))
    return (1 / (math.sqrt(2 * math.pi) * sd)) * exponent


def update_particles_with_motion_model(self):

    motion = self.get_robot_motion()
    d_rot1, d_trans, d_rot2 = motion

    alpha_1 = 0.05
    alpha_2 = 0.05
    alpha_3 = 0.05
    alpha_4 = 0.05
    alpha_5 = 0.05
    alpha_6 = 0.05

    for particle in self.particle_cloud:
        p_pose = particle.pose

        # calculate the new pose of the particle after motion update
        d_rot1_hat = d_rot1 - random.gauss(0, alpha_1 * abs(d_rot1) + alpha_2 * d_trans)
        d_trans_hat = d_trans - random.gauss(0, alpha_3 * d_trans + alpha_4 * (abs(d_rot1) + abs(d_rot2)))
        d_rot2_hat = d_rot2 - random.gauss(0, alpha_1 * abs(d_rot2) + alpha_2 * d_trans)

        theta = get_yaw_from_pose(p_pose)
        x_hat = p_pose.position.x + d_trans_hat * math.cos(theta + d_rot1_hat)
        y_hat = p_pose.position.y + d_trans_hat * math.sin(theta + d_rot1_hat)
        theta_hat = theta + d_rot1_hat + d_rot2_hat + random.gauss(0, alpha_5 * abs(d_rot2_hat) + alpha_6 * d_trans_hat)

        # update the particle with the new pose and a new weight
        p_pose.position.x = x_hat
        p_pose.position.y = y_hat
        q = quaternion_from_euler(0, 0, theta_hat)
        p_pose.orientation.x = q[0]
        p_pose.orientation.y = q[1]
        p_pose.orientation.z = q[2]
        p_pose.orientation.w = q[3]

        particle.pose = p_pose
        particle.w = 1.0

    self.odom_pose_last_motion_update = self.odom_pose